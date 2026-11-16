'use strict';
const assert=require('assert'),config=require('../config.js'),P=require('../visualisation/payload.js'),L=require('../visualisation/payload-lqr.js'),{trial}=require('../run_comparison.js'),old=require('../run_payload.js');
assert.deepStrictEqual(L.gain(0,.4),config.lqr_gain);assert.throws(()=>L.gain(-1,.2));assert.throws(()=>L.gain(.123456,.2));
const condition={initial_tilt_deg:5,push_force_n:5,torque_limit_nm:.7};
for(const load of [{mass_kg:0,height_m:.18},{mass_kg:.75,height_m:.4}]){
 const a=trial('fixed',condition,load,undefined,1001).metrics,b=old.trial('lqr',condition,load,undefined,1001).metrics;
 for(const k of ['outcome','duration_s','recovery_s','peak_tilt_deg','final_position_m','torque_squared_integral','saturation_s','tail_tilt_rms_deg'])assert.equal(a[k],b[k],k);
}
assert.deepStrictEqual(trial('fixed',condition,{mass_kg:0,height_m:.18},undefined,5003,true).rows,trial('exact',condition,{mass_kg:0,height_m:.18},undefined,5003,true).rows);
const load={mass_kg:.65,height_m:.26},noisy={id:'kalman',mode:'kalman',angleNoiseDeg:.75,gyroNoiseDegS:3,delayMs:10};assert.deepStrictEqual(trial('exact',condition,load,noisy,5003),trial('exact',condition,load,noisy,5003));
for(const variant of ['fixed','exact','mass_low','mass_high','height_low','height_high']){const r=trial(variant,condition,load).metrics;assert.equal(r.mass_kg,.65);assert.equal(r.height_m,.26);assert.equal(r.estimated_mass_kg,L.estimate(variant,load).mass_kg);}
console.log('Comparison checks passed: original baseline parity, gain lookup, estimation settings and seeded reproducibility.');

const LiveSession=require('../visualisation/live-model.js');
const live=new LiveSession(config,5,.7,{mode:'ideal'},{mass_kg:.75,height_m:.4},{mode:'payload_lqr',useActual:false,estimatedMass:0,estimatedHeight:.18});
for(let i=0;i<500;i++){if(i===400)live.push(5);live.step();assert.deepStrictEqual(live.robots[0].state,live.robots[1].state);}
const aware=new LiveSession(config,5,.7,{mode:'ideal'},{mass_kg:.75,height_m:.4},{mode:'payload_lqr',useActual:true});for(let i=0;i<500;i++){if(i===400)aware.push(5);aware.step();}assert.notDeepStrictEqual(aware.robots[0].state,aware.robots[1].state);assert.deepStrictEqual(aware.robots[0].p,aware.robots[1].p);
console.log('Live paired LQR check passed: true plants identical; only selected gains differ.');
