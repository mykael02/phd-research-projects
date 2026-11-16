'use strict';
const assert=require('assert'),{Random,Observation,AttitudeKalman,SensedRobot,settings}=require('../visualisation/sensing.js'),{Robot}=require('../model.js'),cfg=require('../config.js');
// Fixed zero-noise/no-delay observations must reproduce the original plant/controller.
for(const mode of ['ideal','direct','kalman'])for(const controller of ['pid','lqr']){const r=new SensedRobot(controller,cfg,5,.7,{mode,angleNoiseDeg:0,gyroNoiseDegS:0,seed:99}),old=new Robot(controller,cfg,5,.7);
 for(let k=0;k<2000;k++){const force=k>=400&&k<440?5:0;r.step(force);old.step(force);assert.deepStrictEqual(r.state,old.state);assert.equal(r.torque,old.torque);}}
// A delayed packet never contains future or current truth after startup.
const o=new Observation({mode:'direct',angleNoiseDeg:0,gyroNoiseDegS:0,delayMs:15,seed:4},.005);
for(let k=0;k<20;k++){const r=o.observe([k,2*k,3*k,4*k],k),j=Math.max(0,k-3);assert.deepStrictEqual(r.estimate,[j,2*j,3*j,4*j]);assert.equal(r.sourceTick,j);assert.strictEqual(o.observe([99,99,99,99],k),r);}
// Repeated startup packet is held; no repeated filter updates / invented history.
const delayed=new Observation({mode:'kalman',angleNoiseDeg:1,gyroNoiseDegS:2,delayMs:20,seed:4},.005);const first=delayed.observe([0,0,.1,.3],0);
for(let k=1;k<=4;k++)assert.deepStrictEqual(delayed.observe([k,k,k,k],k).estimate,first.estimate);
// Changing a plant's hidden truth before a command cannot change that command.
const a=new SensedRobot('lqr',cfg,5,.7,{mode:'direct',delayMs:20,seed:1}),b=new SensedRobot('lqr',cfg,5,.7,{mode:'direct',delayMs:20,seed:1});b.state=[.2,.5,.3,.7];a.step(0);b.step(0);assert.equal(a.torque,b.torque);assert.notDeepStrictEqual(a.state,b.state);
// Fixed seed repeats; changing it changes the acquisition noise.
const p=new Observation({mode:'direct',seed:7},.005),q=new Observation({mode:'direct',seed:7},.005),z=new Observation({mode:'direct',seed:8},.005);assert.deepStrictEqual(p.observe([0,0,0,0],0),q.observe([0,0,0,0],0));assert.notDeepStrictEqual(p.last.measurement,z.observe([0,0,0,0],0).measurement);
// Generator sanity and filter covariance under a long measurement sequence.
const random=new Random(123),values=[];for(let i=0;i<25000;i++)values.push(...random.pair());const mean=values.reduce((a,b)=>a+b)/values.length,variance=values.reduce((a,b)=>a+(b-mean)**2,0)/values.length;assert(Math.abs(mean)<.02);assert(Math.abs(variance-1)<.03);
const filter=new AttitudeKalman(.02,.05,50);let measuredError=0,filteredError=0;for(let k=0;k<5000;k++){const noise=random.pair(),t=k*.005,theta=.1+.04*t,omega=.04;const estimate=filter.update(theta+.02*noise[0],omega+.05*noise[1],.005);if(k>100){measuredError+=(.02*noise[0])**2;filteredError+=(estimate[0]-theta)**2;}assert(filter.a>=0&&filter.c>=0);assert(filter.a*filter.c-filter.b**2>=-1e-14);}
assert(filteredError<measuredError*.25);
for(const bad of [{mode:'wrong'},{delayMs:7},{angleNoiseDeg:-1},{seed:1.5},{gyroNoiseDegS:NaN}])assert.throws(()=>settings(bad));
console.log('Sensing tests passed: perfect-observation parity, causal delay/startup, no truth-fed control, reproducibility, noise statistics, filter covariance and noise reduction on an independent ramp.');
