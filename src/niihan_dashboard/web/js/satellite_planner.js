/* Satellite mission planner. No map image is ever used as evidence of navigability.
 * Browser-only georeferencing. ROS /map remains the authoritative goal safety layer.
 * MapLibre GL JS is loaded separately in index.html. No GNSS commands bypass mission_manager.
 */
const SatelliteGeo = {
    R: 6378137,
    fixValid(fix) {
        return Boolean(fix && fix.available && fix.fresh && fix.lock &&
            Number.isFinite(fix.latitude) && Number.isFinite(fix.longitude) &&
            Math.abs(fix.latitude) < 85 && Math.abs(fix.longitude) <= 180 &&
            (Math.abs(fix.latitude) > 1e-8 || Math.abs(fix.longitude) > 1e-8));
    },
    datumValid(d) {
        return Boolean(d && [d.lat, d.lon, d.x, d.y, d.yaw].every(Number.isFinite) &&
            Math.abs(d.lat) < 85 && Math.abs(d.lon) <= 180 &&
            (Math.abs(d.lat) > 1e-8 || Math.abs(d.lon) > 1e-8));
    },
    toMap(lat, lon, datum) {
        if (!this.datumValid(datum) || !Number.isFinite(lat) || !Number.isFinite(lon) ||
            Math.abs(lat) >= 85 || Math.abs(lon) > 180) throw new Error('Invalid geographic calibration or point.');
        const deltaLon = ((lon - datum.lon + 540) % 360) - 180;
        const east = this.R * Math.cos(datum.lat * Math.PI / 180) * deltaLon * Math.PI / 180;
        const north = this.R * (lat - datum.lat) * Math.PI / 180;
        if (Math.hypot(east, north) > 5000) throw new Error('Waypoint is over 5 km from calibrated origin.');
        // yaw: counter-clockwise angle of ROS map +X measured from geographic east.
        const a = datum.yaw * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
        return {x: datum.x + c * east + s * north, y: datum.y - s * east + c * north};
    },
    toGeo(x, y, datum) {
        if (!this.datumValid(datum) || ![x, y].every(Number.isFinite))
            throw new Error('Invalid map coordinates or geographic calibration.');
        const a = datum.yaw * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
        const dx = x - datum.x, dy = y - datum.y;
        const east = c * dx - s * dy, north = s * dx + c * dy;
        if (Math.hypot(east, north) > 5000) throw new Error('Location is over 5 km from calibrated origin.');
        let lon = datum.lon + east / (this.R * Math.cos(datum.lat * Math.PI / 180)) * 180 / Math.PI;
        lon = ((lon + 540) % 360) - 180;
        return {lat: datum.lat + north / this.R * 180 / Math.PI, lon: lon};
    },
    knownFree(grid, x, y, clearance = 0.45) {
        if (!grid || grid.frame_id !== 'map' || !grid.origin ||
            !Number.isFinite(grid.resolution) || grid.resolution <= 0 ||
            !Number.isInteger(grid.width) || !Number.isInteger(grid.height) ||
            !Array.isArray(grid.data) || grid.data.length !== grid.width * grid.height)
            return {ok: false, reason: 'Valid ROS /map occupancy grid unavailable.'};
        if (![x, y, grid.origin.x, grid.origin.y].every(Number.isFinite))
            return {ok: false, reason: 'Invalid ROS map coordinates.'};
        const a = -(Number(grid.origin.yaw) || 0), c = Math.cos(a), s = Math.sin(a);
        const dx = x - grid.origin.x, dy = y - grid.origin.y;
        const ix = Math.floor((c * dx - s * dy) / grid.resolution);
        const iy = Math.floor((s * dx + c * dy) / grid.resolution);
        const r = Math.ceil(clearance / grid.resolution);
        for (let row = iy - r; row <= iy + r; row++) {
            for (let col = ix - r; col <= ix + r; col++) {
                if (col < 0 || row < 0 || col >= grid.width || row >= grid.height)
                    return {ok: false, reason: 'Outside the currently mapped region.'};
                const value = grid.data[row * grid.width + col];
                if (!Number.isFinite(value) || value < 0)
                    return {ok: false, reason: 'Unknown space: explore this lane first.'};
                if (value >= 65)
                    return {ok: false, reason: 'Occupied or unsafe clearance area.'};
            }
        }
        return {ok: true, reason: 'Known free in /map; Nav2 will still evaluate the route.'};
    }
};

class SatellitePlanner {
    constructor(containerId, submitWaypoint) {
        this.container = document.getElementById(containerId);
        this.submitWaypoint = submitWaypoint;
        this.status = document.getElementById('sat-status');
        this.gnssStatus = document.getElementById('sat-gnss-status');
        this.confirmBox = document.getElementById('sat-confirm');
        this.datum = null;
        this.grid = null;
        this.robot = null;
        this.gnss = null;
        this.connected = false;
        this.canEdit = false;
        this.confirmed = false;
        this.drafts = [];
        this.waypoints = [];
        this.wpMarkers = [];
        this.draftMarkers = [];
        this.map = null;
        this.robotMarker = null;
        this.centeredOnce = false;

        for (const id of ['sat-origin-lat', 'sat-origin-lon', 'sat-map-x', 'sat-map-y', 'sat-map-yaw']) {
            document.getElementById(id).addEventListener('input', () => this.invalidate());
        }
        this.confirmBox.addEventListener('change', () => {
            this.confirmed = false;
            if (!this.confirmBox.checked) {
                this.setStatus('Geographic calibration not confirmed. Waypoint submission disabled.');
                this.renderMarkers();
                return;
            }
            const datum = this.readDatum();
            if (!SatelliteGeo.datumValid(datum)) {
                this.confirmBox.checked = false;
                this.setStatus('Enter a valid datum (latitude/longitude), ROS XY and map yaw.');
                return;
            }
            this.datum = datum;
            this.confirmed = true;
            this.drafts = []; // Old drafts were made in an untrusted coordinate frame.
            this.renderMarkers();
            this.setStatus('Calibration acknowledged. Verify the robot and a SECOND fixed landmark on the map before any submission.');
        });
        document.getElementById('sat-use-fix').addEventListener('click', () => this.anchorAtRobot());
        document.getElementById('sat-center-robot').addEventListener('click', () => this.centerRobot());
        document.getElementById('sat-send-drafts').addEventListener('click', () => this.sendDrafts());
        document.getElementById('sat-clear-drafts').addEventListener('click', () => {
            this.drafts = []; this.renderMarkers();
            this.setStatus('Unsent satellite drafts cleared. Existing mission waypoints unchanged.');
        });
        document.getElementById('sat-street').addEventListener('click', () => this.switchBasemap(false));
        document.getElementById('sat-satellite').addEventListener('click', () => this.switchBasemap(true));
        this.initMap();
    }
    setStatus(value) { this.status.textContent = value; }
    readDatum() {
        const ids = ['sat-origin-lat', 'sat-origin-lon', 'sat-map-x', 'sat-map-y', 'sat-map-yaw'];
        const strings = ids.map(id => document.getElementById(id).value.trim());
        if (strings.some(v => !v)) return null;
        const values = strings.map(Number);
        return {lat: values[0], lon: values[1], x: values[2], y: values[3], yaw: values[4]};
    }
    invalidate() {
        this.confirmed = false; this.datum = null; this.confirmBox.checked = false;
        this.drafts = []; this.renderMarkers();
        this.setStatus('Calibration changed. Old drafts discarded. Check two known landmarks and reconfirm.');
    }
    streetStyle() {
        return {version: 8, sources: {base: {
            type: 'raster', tiles: ['https://tile.openstreetmap.org/{z}/{x}/{y}.png'],
            tileSize: 256, attribution: '© OpenStreetMap contributors'
        }}, layers: [{id: 'base', type: 'raster', source: 'base'}]};
    }
    satelliteStyle(key) {
        return {version: 8, sources: {base: {
            type: 'raster',
            url: 'https://api.maptiler.com/tiles/satellite-v2/tiles.json?key=' + encodeURIComponent(key),
            tileSize: 512, attribution: '© MapTiler'
        }}, layers: [{id: 'base', type: 'raster', source: 'base'}]};
    }
    initMap() {
        if (typeof maplibregl === 'undefined') {
            this.setStatus('MapLibre unavailable. Check internet access. Existing ROS planners remain operational.');
            return;
        }
        try {
            this.map = new maplibregl.Map({
                container: this.container, style: this.streetStyle(),
                center: [0, 0], zoom: 2, maxZoom: 22, attributionControl: true
            });
            this.map.addControl(new maplibregl.NavigationControl(), 'bottom-right');
            const marker = document.createElement('div');
            marker.className = 'sat-robot-marker';
            marker.textContent = 'R';
            marker.title = 'Live GNSS rover position';
            this.robotMarker = new maplibregl.Marker({element: marker});
            this.map.on('click', e => this.addDraft(e.lngLat.lat, e.lngLat.lng));
            this.map.on('error', () => {
                this.setStatus('Map imagery unavailable. Check network, provider key, or tile permissions. ROS navigation unaffected.');
            });
            this.setStatus('Map ready. Enter satellite key to enable imagery; GNSS datum required to submit waypoints.');
        } catch (err) {
            this.setStatus('Satellite map viewer unavailable: ' + err.message);
        }
    }
    switchBasemap(satellite) {
        if (!this.map) return;
        if (!satellite) {
            this.map.setStyle(this.streetStyle());
            this.setStatus('Street map selected. Map tiles are a background, not an obstacle map.');
            return;
        }
        const key = document.getElementById('sat-key').value.trim();
        if (!key) {
            this.setStatus('Enter your own MapTiler API key to display licensed satellite imagery.');
            return;
        }
        this.map.setStyle(this.satelliteStyle(key));
        this.setStatus('Satellite tiles requested. Attribution and usage depend on your MapTiler plan.');
    }
    anchorAtRobot() {
        if (!SatelliteGeo.fixValid(this.gnss) || !this.robot ||
            !this.robot.pose_valid || ![this.robot.x, this.robot.y].every(Number.isFinite)) {
            this.setStatus('Fresh GNSS lock and valid ROS map pose required before anchoring.');
            return;
        }
        const set = (id, n, digits) => {
            document.getElementById(id).value = Number(n).toFixed(digits);
        };
        set('sat-origin-lat', this.gnss.latitude, 8);
        set('sat-origin-lon', this.gnss.longitude, 8);
        set('sat-map-x', this.robot.x, 3);
        set('sat-map-y', this.robot.y, 3);
        this.invalidate();
        this.setStatus('GNSS / ROS robot anchor entered. Calibrate map yaw and SECOND landmark, then confirm.');
    }
    setConnection(connected) {
        this.connected = connected;
        if (!connected) {
            this.canEdit = false;
            this.gnssStatus.textContent = 'Dashboard disconnected: satellite mission submission blocked.';
        }
    }
    setTelemetry(data) {
        const g = data.gnss || {};
        this.gnss = g;
        this.robot = Object.assign({}, data.pose || {}, {pose_valid: data.pose_valid !== false &&
            (data.localization || {}).health === 'OK'});
        this.canEdit = Boolean(this.connected && data.nav2_ready && !data.estop &&
            !['RUNNING', 'CANCELLING', 'PAUSING', 'PAUSED'].includes((data.mission || {}).state));
        if (!SatelliteGeo.fixValid(g)) {
            this.gnssStatus.textContent = 'GNSS unavailable, stale, unlocked or invalid (0,0). Satellite goals blocked.';
            return;
        }
        const accuracy = Number.isFinite(g.horizontal_accuracy_m) ? g.horizontal_accuracy_m.toFixed(2) + ' m' : 'N/A';
        this.gnssStatus.textContent = 'GNSS ' + g.latitude.toFixed(7) + ', ' + g.longitude.toFixed(7) +
            ' | ' + (g.rtk_lock ? 'RTK FIX' : 'NO RTK FIX') +
            ' | accuracy ' + accuracy + (g.simulation ? ' | SIMULATED' : '');
        if (this.robotMarker && this.map) {
            this.robotMarker.setLngLat([g.longitude, g.latitude]).addTo(this.map);
            if (!this.centeredOnce) {
                this.centeredOnce = true;
                this.map.jumpTo({center: [g.longitude, g.latitude], zoom: 18});
            }
        }
    }
    setMap(map) { this.grid = map; }
    setWaypoints(points) {
        this.waypoints = points || [];
        this.renderMarkers();
    }
    centerRobot() {
        if (!SatelliteGeo.fixValid(this.gnss) || !this.map) {
            this.setStatus('No fresh GNSS fix to center on.'); return;
        }
        this.map.jumpTo({center: [this.gnss.longitude, this.gnss.latitude], zoom: 19});
    }
    resize() { if (this.map) this.map.resize(); }
    marker(lat, lon, css, title) {
        const element = document.createElement('div');
        element.className = css;
        element.title = title;
        return new maplibregl.Marker({element: element}).setLngLat([lon, lat]).addTo(this.map);
    }
    renderMarkers() {
        if (!this.map) return;
        for (const marker of this.draftMarkers.concat(this.wpMarkers)) marker.remove();
        this.draftMarkers = []; this.wpMarkers = [];
        for (const d of this.drafts)
            this.draftMarkers.push(this.marker(d.lat, d.lon, 'sat-draft-marker', 'Unsent GPS draft'));
        if (!this.confirmed || !this.datum) return;
        for (const wp of this.waypoints) {
            try {
                const p = SatelliteGeo.toGeo(wp.x, wp.y, this.datum);
                this.wpMarkers.push(this.marker(p.lat, p.lon, 'sat-waypoint-marker',
                    'ROS mission waypoint ' + wp.id));
            } catch (_) { /* Outside calibrated 5km work area, do not render. */ }
        }
    }
    addDraft(lat, lon) {
        if (!this.map) return;
        if (this.drafts.length >= 50) {
            this.setStatus('Maximum 50 unsent satellite drafts reached.'); return;
        }
        if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
        this.drafts.push({lat: lat, lon: lon});
        this.renderMarkers();
        this.setStatus('GPS draft ' + this.drafts.length + ' at ' + lat.toFixed(7) + ', ' +
            lon.toFixed(7) + '. NOT SENT. Click SEND VALID GPS DRAFTS after calibration.');
    }
    readyToSubmit() {
        if (!this.connected || !this.canEdit || !this.robot || !this.robot.pose_valid)
            return 'Dashboard disconnected, Nav2 unavailable, E-stop active or mission not editable.';
        if (!this.confirmed || !SatelliteGeo.datumValid(this.datum))
            return 'Geographic datum not confirmed against two known landmarks.';
        if (!SatelliteGeo.fixValid(this.gnss))
            return 'Fresh, valid GNSS position and lock required.';
        // Physical patrols require RTK; simulated missions may use georeferenced simulated fixes.
        if (!this.gnss.simulation && !this.gnss.rtk_lock)
            return 'Physical mission requires RTK FIX. No GPS targets submitted.';
        try {
            const p = SatelliteGeo.toMap(this.gnss.latitude, this.gnss.longitude, this.datum);
            if (Math.hypot(p.x - this.robot.x, p.y - this.robot.y) > 3)
                return 'GNSS and ROS robot poses differ by more than 3 m. Recalibrate.';
        } catch (err) { return err.message; }
        return '';
    }
    sendDrafts() {
        const blocked = this.readyToSubmit();
        if (blocked) { this.setStatus(blocked); return; }
        if (!this.drafts.length) { this.setStatus('No GPS drafts to send. Click the map first.'); return; }
        let submitted = 0;
        const pending = [];
        for (const d of this.drafts) {
            try {
                const p = SatelliteGeo.toMap(d.lat, d.lon, this.datum);
                const result = SatelliteGeo.knownFree(this.grid, p.x, p.y);
                if (!result.ok || this.submitWaypoint(p.x, p.y) === false) {
                    pending.push(d);
                    continue;
                }
                submitted++;
            } catch (_) { pending.push(d); }
        }
        this.drafts = pending;
        this.renderMarkers();
        this.setStatus(submitted + ' GPS draft(s) submitted to ROS mission manager; ' +
            pending.length + ' withheld (unknown, occupied, out of range or unavailable). ' +
            'Check server responses and mission table. WRITE MISSION and START require separate actions.');
    }
}
if (typeof module !== 'undefined' && module.exports)
    module.exports = {SatelliteGeo, SatellitePlanner};
