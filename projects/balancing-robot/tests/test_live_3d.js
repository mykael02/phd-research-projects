'use strict';
const assert=require('assert'),Session=require('../visualisation/live-model.js'),{Robot}=require('../model.js'),cfg=require('../config.js');
const session=new Session(cfg),reference=['pid','lqr'].map(name=>new Robot(name,cfg,5,.7));
for(let step=0;step<1200;step++){
 if(step===400)session.push(5);if(step===420)session.push(-3);
 const expected=step>=400&&step<420?5:step>=420&&step<460?-3:0;
 assert.equal(session.force,expected);session.step();reference.forEach(r=>r.step(expected,cfg.dt));
 session.robots.forEach((r,i)=>assert.deepStrictEqual(r.state,reference[i].state));
}
assert.equal(session.time,6);assert.equal(session.force,0);
const noForce=new Session(cfg,0,.7);for(let i=0;i<4001;i++)noForce.step();assert.equal(noForce.time,20);assert(noForce.complete);assert.equal(noForce.push(10),false);assert(noForce.robots.every(r=>r.state.every(x=>x===0)));
const falling=new Session(cfg,20,.1);for(let i=0;i<4000;i++)falling.step();assert(falling.robots.every(r=>r.fallen));assert(falling.time<20);const before=JSON.stringify(falling.robots.map(r=>r.state));falling.step();assert.equal(JSON.stringify(falling.robots.map(r=>r.state)),before);
const singleFall=new Session(cfg,-20,.2);for(let i=0;i<400;i++)singleFall.step();assert(singleFall.robots[1].fallen);assert(!singleFall.robots[0].fallen);const frozen=singleFall.robots[1].state.slice();singleFall.push(-10);for(let i=0;i<40;i++)singleFall.step();assert.deepStrictEqual(singleFall.robots[1].state,frozen);assert(singleFall.robots[0].time>singleFall.robots[1].time);
console.log('Live 3D passed: per-step parity with the 2D model, both push directions, exact pulse duration/replacement, 20 s limit, and frozen fallen robots.');
