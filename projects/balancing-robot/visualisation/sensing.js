/* Causal observation pipeline. Estimator accepts packets only, never plant state. */
(function(root){'use strict';
const physics=typeof module!=='undefined'&&module.exports?require('../model.js'):root.Balance;
const RAD=Math.PI/180;
function settings(input={},dt=.005){const p={mode:'ideal',angleNoiseDeg:.5,gyroNoiseDegS:2,delayMs:0,seed:1,accelSigma:50,...input};
 if(!['ideal','direct','kalman'].includes(p.mode))throw new Error('Unknown observation mode');
 for(const k of ['angleNoiseDeg','gyroNoiseDegS','delayMs','accelSigma'])if(!Number.isFinite(p[k])||p[k]<0)throw new Error('Invalid '+k);
 if(!Number.isInteger(p.seed)||p.seed<0||p.seed>4294967295)throw new Error('Seed must be an unsigned 32-bit integer');
 const steps=p.delayMs/(dt*1000);if(Math.abs(steps-Math.round(steps))>1e-8)throw new Error('Delay must be a whole number of control intervals');
 if(p.mode==='ideal'){p.angleNoiseDeg=0;p.gyroNoiseDegS=0;p.delayMs=0;}
 p.delaySteps=Math.round(p.delayMs/(dt*1000));return p;
}
class Random {
 constructor(seed){this.state=seed>>>0;}
 uniform(){this.state=(Math.imul(1664525,this.state)+1013904223)>>>0;return (this.state+.5)/4294967296;}
 pair(){const radius=Math.sqrt(-2*Math.log(this.uniform())),phase=2*Math.PI*this.uniform();return [radius*Math.cos(phase),radius*Math.sin(phase)];}
}
class AttitudeKalman {
 constructor(angleSigma,gyroSigma,accelSigma){this.r0=angleSigma**2;this.r1=gyroSigma**2;this.q=accelSigma**2;this.ready=false;}
 update(theta,omega,dt){
  if(!this.ready||this.r0===0&&this.r1===0){this.theta=theta;this.omega=omega;this.a=this.r0;this.b=0;this.c=this.r1;this.ready=true;return [theta,omega];}
  this.theta+=dt*this.omega;
  let a=this.a+2*dt*this.b+dt*dt*this.c+this.q*dt**4/4,b=this.b+dt*this.c+this.q*dt**3/2,c=this.c+this.q*dt*dt;
  let s=a+this.r0;if(s>0){const error=theta-this.theta;this.theta+=a/s*error;this.omega+=b/s*error;c=Math.max(0,c-b*b/s);b=b*this.r0/s;a=a*this.r0/s;}
  s=c+this.r1;if(s>0){const error=omega-this.omega;this.theta+=b/s*error;this.omega+=c/s*error;a=Math.max(0,a-b*b/s);b=b*this.r1/s;c=c*this.r1/s;}
  this.a=a;this.b=b;this.c=c;return [this.theta,this.omega];
 }
}
class Observation {
 constructor(input,dt){this.dt=dt;this.options=settings(input,dt);this.random=new Random(this.options.seed);this.queue=[];this.lastTick=-1;this.lastSource=-1;this.filter=new AttitudeKalman(this.options.angleNoiseDeg*RAD,this.options.gyroNoiseDegS*RAD,this.options.accelSigma);}
 observe(truth,tick){
  if(tick===this.lastTick)return this.last;
  if(tick!==this.lastTick+1)throw new Error('Sensor acquisition must advance one sample at a time');
  const n=this.random.pair(),p=this.options;
  const packet={tick,state:[truth[0],truth[1],truth[2]+p.angleNoiseDeg*RAD*n[0],truth[3]+p.gyroNoiseDegS*RAD*n[1]]};
  this.queue.push(packet);if(this.queue.length>p.delaySteps+1)this.queue.shift();const delivered=this.queue[0];
  if(delivered.tick!==this.lastSource){const measured=delivered.state;let attitude=measured.slice(2);
   if(p.mode==='kalman')attitude=this.filter.update(measured[2],measured[3],this.dt);
   this.estimate=[measured[0],measured[1],...attitude];this.lastSource=delivered.tick;
  }
  this.lastTick=tick;this.last={measurement:delivered.state.slice(),estimate:this.estimate.slice(),sourceTick:delivered.tick,sourceTime:delivered.tick*this.dt,ageMs:(tick-delivered.tick)*this.dt*1000};return this.last;
 }
}
class SensedRobot {
 constructor(name,config,degrees=5,limit=.7,options={}){
  this.name=name;this.config=config;this.p={...config.parameters,torque_limit:limit};this.state=[0,0,degrees*RAD,0];this.time=0;this.ticks=0;this.torque=0;this.fallen=Math.abs(degrees)>=45;this.peak=Math.abs(degrees);this.maxX=0;this.effort=0;this.saturatedSteps=0;
  this.control=new physics.Robot(name,config,degrees,limit);this.sensor=new Observation(options,config.dt);this.readout=this.sensor.observe(this.state,0);
 }
 get estimate(){return this.readout.estimate;}
 step(push=0,dt=this.config.dt){if(this.fallen)return;if(dt!==this.config.dt)throw new Error('Sensing uses the fixed configured control period');
  this.control.state=this.estimate.slice();const [u,clipped]=this.control.command(dt);this.torque=u;this.saturatedSteps+=Number(clipped);
  this.state=physics.rk4(this.state,u,push,dt,this.p);this.ticks++;this.time=this.ticks*dt;this.effort+=u*u*dt;this.peak=Math.max(this.peak,Math.abs(this.state[2]/RAD));this.maxX=Math.max(this.maxX,Math.abs(this.state[0]));this.fallen=Math.abs(this.state[2])>=Math.PI/4;
  this.readout=this.sensor.observe(this.state,this.ticks);
 }
}
const api={settings,Random,AttitudeKalman,Observation,SensedRobot};root.Sensing=api;if(typeof module!=='undefined'&&module.exports)module.exports=api;
})(typeof globalThis!=='undefined'?globalThis:this);
