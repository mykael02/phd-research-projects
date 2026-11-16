/* Fixed-step live session, using the exact same plant and controllers as the 2D lab. */
(function(root){'use strict';
const physics=typeof module!=='undefined'&&module.exports?require('../model.js'):root.Balance;
const sensing=typeof module!=='undefined'&&module.exports?require('./sensing.js'):root.Sensing;
const payloadModel=typeof module!=='undefined'&&module.exports?require('./payload.js'):root.Payload;
const lqrModel=typeof module!=='undefined'&&module.exports?require('./payload-lqr.js'):root.PayloadLQR;
class LiveSession {
  constructor(config,tilt=5,limit=.7,options={},payload={mass_kg:0,height_m:.18},comparison={mode:'pid_lqr'}){
    const loaded=payloadModel.configure(config,payload.mass_kg,payload.height_m);this.config=loaded.config;this.payload=loaded.payload;this.dt=config.dt;this.duration=20;this.ticks=0;this.pulseTicks=0;this.pushForce=0;this.options=sensing.settings(options,this.dt);this.initialTilt=tilt;this.torqueLimit=limit;this.events=[];
    if(!['pid_lqr','payload_lqr'].includes(comparison.mode))throw new Error('Unknown controller comparison');
    const estimated=comparison.useActual!==false?{mass_kg:payload.mass_kg,height_m:payload.height_m}:{mass_kg:comparison.estimatedMass,height_m:comparison.estimatedHeight};
    this.comparison={mode:comparison.mode,estimated_payload:estimated,use_actual:comparison.useActual!==false};
    const aware=comparison.mode==='payload_lqr'?{...this.config,lqr_gain:lqrModel.gain(estimated.mass_kg,estimated.height_m)}:this.config;
    this.labels=comparison.mode==='payload_lqr'?['Fixed LQR','Payload-aware LQR']:['PID-type','LQR'];
    this.robots=comparison.mode==='payload_lqr'?[new sensing.SensedRobot('lqr',this.config,tilt,limit,this.options),new sensing.SensedRobot('lqr',aware,tilt,limit,this.options)]:['pid','lqr'].map(name=>new sensing.SensedRobot(name,this.config,tilt,limit,this.options));
    this.controllerGains=this.robots.map(r=>r.name==='lqr'?r.config.lqr_gain.slice():null);this.history=[];this.record();
  }
  get time(){return this.ticks*this.dt;}
  get complete(){return this.time>=this.duration-1e-9||this.robots.every(r=>r.fallen);}
  get force(){return this.pulseTicks>0&&!this.complete?this.pushForce:0;}
  push(force){if(this.complete)return false;this.pushForce=force;this.pulseTicks=Math.round(.2/this.dt);this.events.push({time:this.time,force_n:force,duration_s:.2});return true;}
  record(){this.history.push({time:this.time,robots:this.robots.map(r=>({state:r.state.slice(),estimate:r.estimate.slice(),measurement:r.readout.measurement.slice(),sourceTime:r.readout.sourceTime,fallen:r.fallen}))});}
  step(){if(this.complete)return;const force=this.force;this.robots.forEach(robot=>robot.step(force,this.dt));this.ticks++;this.pulseTicks=Math.max(0,this.pulseTicks-1);this.record();}
}
root.LiveSession=LiveSession;if(typeof module!=='undefined'&&module.exports)module.exports=LiveSession;
})(typeof globalThis!=='undefined'?globalThis:this);
