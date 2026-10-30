/* Site Robotics visual site planner.
 * Loads a user-selected, self-contained GLB locally; nothing is uploaded.
 * The model is VISUAL ONLY and cannot override Nav2 or the geofence.
 * THREE r128 and matching GLTFLoader are provided by index.html.
 */
class SitePlanner {
    constructor(containerId, callback) {
        this.container=document.getElementById(containerId);
        this.callback=callback;
        this.ready=false;
        this.confirmed=false;
        this.waypoints=[];
        this.robot={x:0,y:0,z:0};
        this.pickPoint=null;
        this.map=null;
        this.drafts=[]; // Visual-only candidates; never drive to unknown cells.
        this.scene=new THREE.Scene();
        this.scene.background=new THREE.Color(0x1d2025);
        this.camera=new THREE.OrthographicCamera(-30,30,20,-20,0.1,10000);
        this.camera.up.set(0,1,0);
        this.camera.position.set(0,0,150);
        this.camera.lookAt(0,0,0);
        this.renderer=new THREE.WebGLRenderer({antialias:true});
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));
        this.container.appendChild(this.renderer.domElement);
        this.controls=new THREE.OrbitControls(this.camera,this.renderer.domElement);
        this.controls.enableRotate=false;
        this.controls.enableDamping=true;
        this.controls.enablePan=true;
        this.controls.enableZoom=true;
        this.scene.add(new THREE.AmbientLight(0xffffff,.9));
        const sun=new THREE.DirectionalLight(0xffffff,.85);
        sun.position.set(30,-20,80);this.scene.add(sun);
        this.siteRoot=new THREE.Group();
        this.scene.add(this.siteRoot);
        this.axisConversion=new THREE.Group();
        this.axisConversion.rotation.x=Math.PI/2; // glTF Y-up => ROS Z-up
        this.siteRoot.add(this.axisConversion);
        this.markers=new THREE.Group();this.scene.add(this.markers);
        const arrowShape=new THREE.Shape();
        arrowShape.moveTo(1.6,0);arrowShape.lineTo(-.9,.65);arrowShape.lineTo(-.6,0);arrowShape.lineTo(-.9,-.65);arrowShape.closePath();
        this.robotMarker=new THREE.Mesh(new THREE.ShapeGeometry(arrowShape),new THREE.MeshBasicMaterial({color:0x1faaff,depthTest:false,side:THREE.DoubleSide}));
        this.robotMarker.renderOrder=99;
        this.markers.add(this.robotMarker);
        const ring=new THREE.RingGeometry(1.9,2.25,36);
        this.robotRing=new THREE.Mesh(ring,new THREE.MeshBasicMaterial({color:0x00e5ff,side:THREE.DoubleSide,depthTest:false,depthWrite:false}));
        this.robotRing.renderOrder=100;
        this.markers.add(this.robotRing);
        const label=document.createElement('canvas');
        label.width=256;label.height=64;
        const pen=label.getContext('2d');
        pen.fillStyle='rgba(0,12,22,.86)';pen.fillRect(0,0,256,64);
        pen.fillStyle='#ffffff';pen.font='bold 36px sans-serif';pen.textAlign='center';
        pen.fillText('ROBOT',128,44);
        this.robotLabel=new THREE.Sprite(new THREE.SpriteMaterial({
            map:new THREE.CanvasTexture(label),depthTest:false,depthWrite:false,transparent:true
        }));
        this.robotLabel.renderOrder=102;
        this.markers.add(this.robotLabel);
        this.wpMarkers=new THREE.Group();this.markers.add(this.wpMarkers);
        this.draftMarkers=new THREE.Group();this.markers.add(this.draftMarkers);
        this.raycaster=new THREE.Raycaster();
        this.groundPlane=new THREE.Plane(new THREE.Vector3(0,0,1),0);
        this.mouse=new THREE.Vector2();
        this.status=document.getElementById('site-planner-status');
        this.confirm=document.getElementById('site-align-confirm');
        this.file=document.getElementById('site-glb-file');
        this.file.addEventListener('change',event=>{
            const glb=event.target.files&&event.target.files[0];
            if(glb)this.load(glb);
        });
        for(const id of ['site-offset-x','site-offset-y','site-yaw','site-scale']) {
            document.getElementById(id).addEventListener('input',()=>this.applyAlignment());
        }
        this.confirm.addEventListener('change',()=>{
            this.confirmed=this.confirm.checked&&this.ready;
            this.describe();
        });
        document.getElementById('site-fit').addEventListener('click',()=>this.fit());
        document.getElementById('site-center-robot').addEventListener('click',()=>this.centerRobot());
        document.getElementById('site-zoom-in').addEventListener('click',()=>this.zoom(1.5));
        document.getElementById('site-zoom-out').addEventListener('click',()=>this.zoom(1/1.5));
        document.getElementById('site-send-drafts').addEventListener('click',()=>this.sendValidDrafts());
        const canvas=this.renderer.domElement;
        let down=null,dragged=false;
        canvas.addEventListener('pointerdown',e=>{
            if(e.button!==0)return;
            down={x:e.clientX,y:e.clientY};dragged=false;
        });
        canvas.addEventListener('pointermove',e=>{
            if(down&&Math.hypot(e.clientX-down.x,e.clientY-down.y)>5)dragged=true;
        });
        canvas.addEventListener('pointerup',e=>{
            if(down&&!dragged)this.pick(e);
            down=null;
        });
        canvas.addEventListener('pointercancel',()=>{down=null;});
        window.addEventListener('resize',()=>this.resize());
        this.resize();
        this.animate=()=>{requestAnimationFrame(this.animate);this.controls.update();this.updateMarkerSize();this.renderer.render(this.scene,this.camera);};
        this.animate();
        this.describe();
    }
    setStatus(message) {this.status.textContent=message;}
    describe() {
        if(!this.ready)this.setStatus('Select your seaport .glb file to preview it. No upload is performed.');
        else if(!this.confirmed)this.setStatus('PREVIEW ONLY — align GLB with ROS coordinates, then confirm before adding waypoints.');
        else this.setStatus('Click a known free lane to add a waypoint. Unknown areas become visual drafts (not sent). Drafts: '+this.drafts.length+'.');
    }
    load(file) {
        this.confirm.checked=false;this.confirmed=false;this.ready=false;
        if(!/\.glb$/i.test(file.name)){
            this.setStatus('Only self-contained .glb files are supported.');return;
        }
        if(!THREE.GLTFLoader){
            this.setStatus('GLTFLoader unavailable. Check network access to the loader script.');return;
        }
        this.setStatus('Reading '+file.name+' locally...');
        const reader=new FileReader();
        reader.onerror=()=>this.setStatus('Unable to read the selected file.');
        reader.onload=()=>{
            const loader=new THREE.GLTFLoader();
            loader.parse(reader.result,'',gltf=>{
                while(this.axisConversion.children.length){
                    const old=this.axisConversion.children[0];
                    this.axisConversion.remove(old);
                    old.traverse(n=>{
                        if(n.geometry)n.geometry.dispose();
                        const mats=Array.isArray(n.material)?n.material:[n.material];
                        mats.forEach(m=>{if(m)m.dispose();});
                    });
                }
                this.axisConversion.add(gltf.scene);
                this.ready=true;
                this.applyAlignment();
                this.fit();
                this.describe();
            },err=>this.setStatus('GLB parse failed: '+err.message));
        };
        reader.readAsArrayBuffer(file);
    }
    applyAlignment(){
        const val=id=>Number(document.getElementById(id).value);
        const values=['site-offset-x','site-offset-y','site-yaw','site-scale'].map(val);
        if(!values.every(Number.isFinite)||values[3]<=0||values[3]>100){
            this.confirmed=false;this.confirm.checked=false;
            this.setStatus('Enter finite offsets, rotation, and a positive scale ≤ 100.');return;
        }
        this.siteRoot.position.set(values[0],values[1],0);
        this.siteRoot.rotation.z=values[2]*Math.PI/180;
        this.siteRoot.scale.setScalar(values[3]);
        this.confirm.checked=false;this.confirmed=false;
        this.describe();
    }
    setMap(map){this.map=map;}
    setRobot(pose){
        if(!pose||![pose.x,pose.y].every(Number.isFinite))return;
        this.robot={x:pose.x,y:pose.y,z:pose.z||0,yaw:pose.yaw||0};
        this.drawMarkers();
    }
    setWaypoints(points){this.waypoints=points||[];this.drawMarkers();}
    drawMarkers(){
        while(this.markers.children.length>1){
            const o=this.markers.children[1];this.markers.remove(o);
            if(o.geometry)o.geometry.dispose();
            if(o.material)o.material.dispose();
        }
        this.robotMarker.position.set(this.robot.x,this.robot.y,1);
        this.robotMarker.rotation.set(0,0,this.robot.yaw||0);
        const markerMaterial=new THREE.MeshBasicMaterial({color:0x49e699,depthTest:false});
        for(const wp of this.waypoints){
            if(![wp.x,wp.y].every(Number.isFinite))continue;
            const m=new THREE.Mesh(new THREE.SphereGeometry(.65,10,8),markerMaterial);
            m.position.set(wp.x,wp.y,2);m.renderOrder=101;
            this.markers.add(m);
        }
        // Reuse material carefully: the group is redrawn infrequently.
    }
    // Convert world coordinates into the current ROS occupancy grid. A known,
    // free target is required; the GLB visual geometry alone is not a safety map.
    targetIsKnownFree(x,y) {
        const m=this.map;
        if(!m||!m.origin||!Array.isArray(m.data)||!m.resolution||!m.width||!m.height)
            return {ok:false,reason:'2D navigation map unavailable; cannot validate this waypoint.'};
        const yaw=m.origin.yaw||0,c=Math.cos(-yaw),s=Math.sin(-yaw);
        const dx=x-m.origin.x,dy=y-m.origin.y;
        const ix=Math.floor((c*dx-s*dy)/m.resolution);
        const iy=Math.floor((s*dx+c*dy)/m.resolution);
        const radius=Math.ceil(.45/m.resolution);
        for(let dy=-radius;dy<=radius;dy++)for(let dx=-radius;dx<=radius;dx++){
            const u=ix+dx,v=iy+dy;
            if(u<0||v<0||u>=m.width||v>=m.height)
                return {ok:false,reason:'Outside currently mapped navigation area.'};
            const value=m.data[v*m.width+u];
            if(value<0)return {ok:false,reason:'Unmapped area — explore closer before assigning this point.'};
            if(value>=65)return {ok:false,reason:'Obstacle or inflated footprint area; choose an open lane.'};
        }
        return {ok:true};
    }
    pick(e) {
        if(!this.ready||!this.confirmed){
            this.describe();return;
        }
        const rect=this.renderer.domElement.getBoundingClientRect();
        this.mouse.set(((e.clientX-rect.left)/rect.width)*2-1,
                       -((e.clientY-rect.top)/rect.height)*2+1);
        this.raycaster.setFromCamera(this.mouse,this.camera);
        const p=new THREE.Vector3();
        if(!this.raycaster.ray.intersectPlane(this.groundPlane,p))return;
        const test=this.targetIsKnownFree(p.x,p.y);
        if(!test.ok){this.setStatus(test.reason);return;}
        this.setStatus('Waypoint candidate: X='+p.x.toFixed(2)+' Y='+p.y.toFixed(2)+' — requesting dashboard insertion.');
        this.callback(p.x,p.y);
    }
    fit(){
        if(!this.ready)return;
        this.siteRoot.updateMatrixWorld(true);
        const bounds=new THREE.Box3().setFromObject(this.siteRoot);
        if(bounds.isEmpty())return;
        const size=bounds.getSize(new THREE.Vector3()),mid=bounds.getCenter(new THREE.Vector3());
        this.controls.target.set(mid.x,mid.y,0);
        const w=Math.max(size.x,10)*1.2,h=Math.max(size.y,10)*1.2;
        const aspect=Math.max(1,this.container.clientWidth)/Math.max(1,this.container.clientHeight);
        const half=Math.max(h/2,w/(2*aspect));
        this.camera.left=-half*aspect;this.camera.right=half*aspect;
        this.camera.top=half;this.camera.bottom=-half;
        this.camera.position.set(mid.x,mid.y,150);
        this.camera.updateProjectionMatrix();
        this.controls.update();
    }
    resize(){
        const w=this.container.clientWidth,h=this.container.clientHeight;
        if(!w||!h)return;
        const center=(this.camera.left+this.camera.right)/2;
        const halfH=(this.camera.top-this.camera.bottom)/2;
        const aspect=w/h;
        this.camera.left=center-halfH*aspect;this.camera.right=center+halfH*aspect;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w,h);
    }
}