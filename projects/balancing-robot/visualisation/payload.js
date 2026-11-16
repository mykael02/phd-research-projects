/* Rigid, centred cuboid payload. SI units; height measured along body from axle. */
(function(root){'use strict';
const shape=Object.freeze({width:.08,height:.06,depth:.12});
function combine(parameters,mass=0,height=.18){
 if(!Number.isFinite(mass)||mass<0||mass>1.5)throw new Error('Payload mass must be between 0 and 1.5 kg');
 if(!Number.isFinite(height)||height<.1||height>.4)throw new Error('Payload height must be between 0.10 and 0.40 m');
 const m=parameters.body_mass,l=parameters.length,I=parameters.body_inertia;
 const total=m+mass,centre=mass===0?l:(m*l+mass*height)/total;
 const own=mass*(shape.width**2+shape.height**2)/12;
 const inertia=mass===0?I:I+m*(l-centre)**2+own+mass*(height-centre)**2;
 return {parameters:{...parameters,body_mass:total,length:centre,body_inertia:inertia},
  payload:{mass_kg:mass,height_m:height,shape:{...shape},own_inertia_kg_m2:own},
  axle_inertia_kg_m2:inertia+total*centre**2};
}
function configure(config,mass=0,height=.18){const loaded=combine(config.parameters,mass,height);return {...loaded,config:{...config,parameters:loaded.parameters}};}
const api={shape,combine,configure};root.Payload=api;if(typeof module!=='undefined'&&module.exports)module.exports=api;
})(typeof globalThis!=='undefined'?globalThis:this);
