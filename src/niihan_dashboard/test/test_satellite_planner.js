/* Node-only regression tests: node test/test_satellite_planner.js */
const assert = require('assert');
const {SatelliteGeo: g} = require('../web/js/satellite_planner.js');
const datum = {lat: 18.52, lon: 73.85, x: -2.5, y: 8, yaw: 27};
assert(g.datumValid(datum));
assert(!g.datumValid({...datum, lat: 0, lon: 0}));
assert(!g.fixValid({available: true, fresh: true, lock: true, latitude: 0, longitude: 0}));
assert(g.fixValid({available: true, fresh: true, lock: true, latitude: 18.52, longitude: 73.85}));
const geo = {lat: 18.52012, lon: 73.85016};
const xy = g.toMap(geo.lat, geo.lon, datum);
const back = g.toGeo(xy.x, xy.y, datum);
assert(Math.abs(back.lat - geo.lat) < 1e-8);
assert(Math.abs(back.lon - geo.lon) < 1e-8);
assert.throws(() => g.toMap(19.3, 74.2, datum), /5 km/);
const yaw90 = {lat: 18.52, lon: 73.85, x: 0, y: 0, yaw: 90};
const eastPoint = g.toMap(18.52, 73.85001, yaw90);
assert(Math.abs(eastPoint.x) < 0.01 && eastPoint.y < -1);
const grid = {
    frame_id: 'map', origin: {x: -2, y: -2, yaw: 0},
    resolution: 0.1, width: 80, height: 80, data: Array(6400).fill(0)
};
assert(g.knownFree(grid, 0, 0).ok);
grid.data[20 * 80 + 20] = -1;
assert(!g.knownFree(grid, 0, 0).ok);
grid.data[20 * 80 + 20] = 100;
assert(!g.knownFree(grid, 0, 0).ok);
assert(!g.knownFree(grid, 200, 0).ok);
assert(!g.knownFree(null, 0, 0).ok);
console.log('Satellite planner georeferencing / occupancy tests passed.');
