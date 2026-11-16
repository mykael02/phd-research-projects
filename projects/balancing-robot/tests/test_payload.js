'use strict';
const assert=require('assert'),Payload=require('../visualisation/payload.js'),config=require('../config.js'),physics=require('../model.js'),{SensedRobot}=require('../visualisation/sensing.js'),{trial,loads}=require('../run_payload.js');
assert.equal(loads().length,21);assert.deepStrictEqual(Payload.configure(config,0,.4).config,config);
const before=JSON.stringify(config);
for(const {mass_kg:m,height_m:h} of loads()){
 const {parameters:p,axle_inertia_kg_m2:J}=Payload.combine(config.parameters,m,h),b=config.parameters;
 const expectedJ=b.body_inertia+b.body_mass*b.length**2+m*h*h+m*(.08**2+.06**2)/12;
 assert(Math.abs(J-expectedJ)<1e-14);assert(Math.abs(p.body_mass*p.length-(b.body_mass*b.length+m*h))<1e-14);
 assert(p.body_inertia>0);assert(p.length>=Math.min(h,b.length)-1e-14&&p.length<=Math.max(h,b.length)+1e-14);
 for(const t of [-.7,0,.7]){const a=p.base_mass+p.body_mass+2*p.wheel_inertia/p.radius**2,beta=p.body_mass*p.length*Math.cos(t);assert(a*J-beta*beta>0);}
}
assert.equal(JSON.stringify(config),before);
for(const [m,h] of [[-1,.18],[2,.18],[NaN,.18],[1,.09],[1,.41],[1,Infinity]])assert.throws(()=>Payload.combine(config.parameters,m,h));
for(const name of ['pid','lqr']){
 const a=new SensedRobot(name,config),b=new SensedRobot(name,Payload.configure(config,0,.1).config);
 for(let i=0;i<2000;i++){const force=i>=400&&i<440?5:0;a.step(force);b.step(force);assert.deepStrictEqual(a.state,b.state);assert.equal(a.torque,b.torque);}
 const c={initial_tilt_deg:5,push_force_n:5,torque_limit_nm:.7},load={mass_kg:.75,height_m:.4},profile={id:'kalman',mode:'kalman',angleNoiseDeg:.5,gyroNoiseDegS:2,delayMs:10};
 assert.deepStrictEqual(trial(name,c,load,profile,1001),trial(name,c,load,profile,1001));
 assert.notDeepStrictEqual(trial(name,c,load,profile,1001),trial(name,c,load,profile,2002));
 const rest=trial(name,{initial_tilt_deg:0,push_force_n:0,torque_limit_nm:.7},load).metrics;assert.equal(rest.outcome,'recovered');assert.equal(rest.recovery_s,0);assert.equal(rest.tail_tilt_rms_deg,0);
}
console.log('Payload checks passed: composite inertia, positive mass matrix, zero-load parity, no mutation, validation and seeded trial repeatability.');
