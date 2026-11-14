// Run: node test/test_site_planner_drafts.js
// Pure interaction tests: no WebGL, DOM renderer, ROS or Gazebo needed.
const assert = require('node:assert/strict');
const SitePlanner = require('../web/js/site_planner.js');

function stub() {
    let sent = 0, drawn = 0;
    const planner = Object.create(SitePlanner.prototype);
    planner.ready = true;
    planner.confirmed = false;
    planner.anchorRobotPending = false;
    planner.previewPins = [];
    planner.drafts = [];
    planner.map = null;
    planner.mouse = {set() {}};
    planner.camera = {};
    planner.groundPlane = {};
    planner.renderer = {domElement:{getBoundingClientRect(){return {left:0,top:0,width:200,height:100};}}};
    planner.raycaster = {
        setFromCamera() {},
        ray:{intersectPlane(_plane, point) {point.x=5;point.y=5;return point;}}
    };
    planner.setStatus = message => {planner.message = message;};
    planner.drawMarkers = () => {drawn++;};
    planner.callback = () => {sent++;return true;};
    return {planner, get sent(){return sent;},get drawn(){return drawn;}};
}
// THREE.Vector3 is only instantiated in pick; avoid constructing renderer.
global.THREE = {Vector3: class {}};
function click(p) {p.pick({clientX:100,clientY:50});}
let t=stub();
click(t.planner);
assert.equal(t.planner.previewPins.length,1);
assert.equal(t.sent,0);
assert.match(t.planner.message,/NOT SENT/);

t.planner.clearVisualPins();
assert.equal(t.planner.previewPins.length,0);
assert.match(t.planner.message,/Saved mission waypoints were NOT changed/);

t=stub();
t.planner.confirmed=true;
t.planner.map={width:20,height:20,resolution:1,origin:{x:0,y:0,yaw:0},data:Array(400).fill(0)};
click(t.planner);
assert.equal(t.sent,1);
assert.equal(t.planner.drafts.length,0);

t=stub();
t.planner.confirmed=true;
click(t.planner); // no live occupancy grid: submit for planning, not automatic movement
assert.equal(t.sent,1);
assert.equal(t.planner.drafts.length,0);
assert.match(t.planner.message,/UNVERIFIED/);

t=stub();
t.planner.confirmed=true;
t.planner.map={width:20,height:20,resolution:1,origin:{x:0,y:0,yaw:0},data:Array(400).fill(-1)};
click(t.planner); // unmapped target is a mission coordinate, not a drive command
assert.equal(t.sent,1);
assert.match(t.planner.message,/UNVERIFIED/);

t=stub();
t.planner.confirmed=true;
t.planner.map={width:20,height:20,resolution:1,origin:{x:0,y:0,yaw:0},data:Array(400).fill(0)};
t.planner.map.data[4*20+4]=-1; // one unknown cell inside the 0.45 m clearance ring
click(t.planner);
assert.equal(t.sent,1);
assert.match(t.planner.message,/UNVERIFIED/);

t=stub();
t.planner.confirmed=true;
t.planner.map={width:20,height:20,resolution:1,origin:{x:0,y:0,yaw:0},data:Array(400).fill(0)};
t.planner.map.data[5*20+5]=-1; // unknown must NOT hide a known obstacle nearby
t.planner.map.data[6*20+5]=100;
click(t.planner);
assert.equal(t.sent,0);
assert.equal(t.planner.drafts.length,1);

t=stub();
t.planner.confirmed=true;
t.planner.map={width:20,height:20,resolution:1,origin:{x:0,y:0,yaw:0},data:Array(400).fill(100)};
click(t.planner); // occupied area: visual-only draft
assert.equal(t.sent,0);
assert.equal(t.planner.drafts.length,1);

t=stub();
t.planner.anchorRobotPending=true;
t.planner.robot={x:11,y:12};
const originalDocument=global.document;
global.document={getElementById(id){
    return {'site-offset-x':{value:'0'},'site-offset-y':{value:'0'}}[id];
}};
t.planner.applyAlignment=()=>{t.planner.confirmed=false;};
click(t.planner);
assert.equal(t.planner.anchorRobotPending,false);
assert.equal(t.planner.previewPins.length,0);
assert.equal(t.sent,0);
global.document=originalDocument;
console.log('PASS site planner GLB waypoint submission, obstacle checks, alignment and safety guards');
