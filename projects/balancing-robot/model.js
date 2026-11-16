/* Browser/Node implementation of balance.py. State: x, velocity, tilt, tilt rate. */
(function (root) {
  'use strict';
  const clamp = (v, limit) => Math.max(-limit, Math.min(limit, v));
  function derivative(s, torque, push, p) {
    const [,v,theta,omega]=s, sin=Math.sin(theta), cos=Math.cos(theta);
    const a=p.base_mass+p.body_mass+2*p.wheel_inertia/p.radius**2;
    const b=p.body_mass*p.length*cos, d=p.body_inertia+p.body_mass*p.length**2;
    const f=torque/p.radius-p.translation_drag*v+p.body_mass*p.length*sin*omega**2+push;
    const h=p.body_mass*p.gravity*p.length*sin-torque-p.rotation_drag*omega+push*p.length*cos;
    const det=a*d-b*b;
    return [v,(d*f-b*h)/det,omega,(a*h-b*f)/det];
  }
  function rk4(s,u,f,dt,p) {
    const add=(k,w)=>s.map((y,i)=>y+dt*k[i]*w);
    const k1=derivative(s,u,f,p),k2=derivative(add(k1,.5),u,f,p),k3=derivative(add(k2,.5),u,f,p),k4=derivative(add(k3,1),u,f,p);
    return s.map((y,i)=>y+dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6);
  }
  class Robot {
    constructor(name,config,degrees=5,limit=config.parameters.torque_limit) {
      this.name=name;this.config=config;this.p={...config.parameters,torque_limit:limit};
      this.state=[0,0,degrees*Math.PI/180,0];this.integral=0;this.time=0;this.torque=0;
      this.fallen=Math.abs(degrees)>=45;this.peak=Math.abs(degrees);this.maxX=0;this.effort=0;this.saturatedSteps=0;
    }
    command(dt) {
      const [x,v,theta,omega]=this.state, limit=this.p.torque_limit;
      let raw=0;
      if(this.name==='lqr') raw=-this.config.lqr_gain.reduce((sum,k,i)=>sum+k*this.state[i],0);
      else if(this.name==='pid') {
        const g=this.config.pid,candidate=clamp(this.integral+theta*dt,g.integral_limit);
        const other=g.kp*theta+g.kd*omega+g.kx*x+g.kv*v;
        const trial=other+g.ki*candidate;
        if(Math.abs(trial)<=limit||trial*theta<0)this.integral=candidate;
        raw=other+g.ki*this.integral;
      }
      return [clamp(raw,limit),Math.abs(raw)>limit];
    }
    step(push=0,dt=this.config.dt) {
      if(this.fallen)return;
      const [u,clipped]=this.command(dt);this.torque=u;this.saturatedSteps+=Number(clipped);
      this.state=rk4(this.state,u,push,dt,this.p);this.time+=dt;this.effort+=u*u*dt;
      this.peak=Math.max(this.peak,Math.abs(this.state[2]*180/Math.PI));this.maxX=Math.max(this.maxX,Math.abs(this.state[0]));
      this.fallen=Math.abs(this.state[2])>=Math.PI/4;
    }
  }
  const api={derivative,rk4,Robot};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.Balance=api;
})(typeof globalThis!=='undefined'?globalThis:this);
