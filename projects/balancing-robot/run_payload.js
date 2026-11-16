'use strict';
const fs=require('fs'),path=require('path'),crypto=require('crypto');
const config=require('./config.js'),Payload=require('./visualisation/payload.js'),{SensedRobot}=require('./visualisation/sensing.js');
const protocol=JSON.parse(fs.readFileSync(path.join(__dirname,'payload_protocol.json'),'utf8')),OUT=path.join(__dirname,'results/payload');
function loads(){return protocol.payload_mass_kg.flatMap(m=>m===0?[{mass_kg:0,height_m:.18}]:protocol.payload_height_m.map(h=>({mass_kg:m,height_m:h})));}
function trial(controller,condition,load,profile={id:'ideal',mode:'ideal'},seed=1001,capture=false){
 const loaded=Payload.configure(config,load.mass_kg,load.height_m),c=loaded.config;
 const r=new SensedRobot(controller,c,condition.initial_tilt_deg,condition.torque_limit_nm,{...profile,seed,accelSigma:50});
 const reference=condition.push_force_n===0?0:2.2,rows=[];let lastBad=null,tail=[0,0,0],tailN=0;
 function record(){const s=r.state,t=r.time;if(t>=reference-1e-10&&(Math.abs(s[0])>.03||Math.abs(s[1])>.03||Math.abs(s[2])>Math.PI/180))lastBad=t;
  if(t>=9-1e-10){for(let i=0;i<3;i++)tail[i]+=s[i]**2;tailN++;}
  if(capture)rows.push({time:t,state:s.slice(),estimate:r.estimate.slice(),torque:r.torque});
 }
 record();while(r.time<10-1e-10&&!r.fallen){const f=r.time>=2-1e-10&&r.time<2.2-1e-10?condition.push_force_n:0;r.step(f);record();}
 const settled=Math.max(reference,lastBad===null?reference:lastBad+c.dt),recovery=!r.fallen&&r.time-settled>=1-1e-10?settled-reference:null,full=r.time>=10-1e-10&&!r.fallen;
 const metrics={controller,profile:profile.id,seed,mass_kg:load.mass_kg,height_m:load.height_m,initial_tilt_deg:condition.initial_tilt_deg,push_force_n:condition.push_force_n,torque_limit_nm:condition.torque_limit_nm,
  body_mass_kg:c.parameters.body_mass,com_m:c.parameters.length,body_inertia_kg_m2:c.parameters.body_inertia,axle_inertia_kg_m2:loaded.axle_inertia_kg_m2,
  outcome:r.fallen?'fallen':recovery===null?'unsettled':'recovered',duration_s:r.time,recovery_s:recovery,peak_tilt_deg:r.peak,max_displacement_m:r.maxX,final_position_m:r.state[0],torque_squared_integral:r.effort,saturation_s:r.saturatedSteps*c.dt,
  tail_tilt_rms_deg:full?Math.sqrt(tail[2]/tailN)*180/Math.PI:null,tail_position_rms_m:full?Math.sqrt(tail[0]/tailN):null,tail_speed_rms_m_s:full?Math.sqrt(tail[1]/tailN):null};
 return {metrics,rows};
}
function main(){fs.mkdirSync(OUT,{recursive:true});const sourceFiles=['model.js','config.js','visualisation/sensing.js','visualisation/payload.js','run_payload.js','payload_protocol.json','run_payload_audit.js'];
 const metadata={protocol,node:process.version,config,source_sha256:Object.fromEntries(sourceFiles.map(f=>[f,crypto.createHash('sha256').update(fs.readFileSync(path.join(__dirname,f))).digest('hex')]))};
 fs.writeFileSync(path.join(OUT,'protocol_snapshot.json'),JSON.stringify(metadata,null,2)+'\n');const records=[],examples={};
 for(const load of loads())for(const controller of protocol.controllers){for(const tilt of protocol.ideal_grid.tilt_deg)for(const force of protocol.ideal_grid.force_n)for(const limit of protocol.ideal_grid.torque_limit_nm){
  const condition={initial_tilt_deg:tilt,push_force_n:force,torque_limit_nm:limit};const save=tilt===5&&force===5&&limit===.7&&[0,.75,1.5].includes(load.mass_kg)&&(load.mass_kg===0||load.height_m===.4);
  const result=trial(controller,condition,load,undefined,1001,save);records.push({phase:'ideal_grid',scenario:'grid',...result.metrics});if(save)examples[controller+'_'+load.mass_kg]=result;
 }}
 console.log(`Ideal grid complete: ${records.length} trials`);
 for(const load of protocol.sensing_loads){for(const scenario of protocol.scenarios)for(const controller of protocol.controllers)for(const profile of protocol.sensing_profiles)for(let i=0;i<30;i++)records.push({phase:'sensing',scenario:scenario.id,...trial(controller,scenario,load,profile,1001+104729*i).metrics});console.log(`Sensing complete through load ${load.mass_kg} kg at ${load.height_m} m: ${records.length} trials`);}
 fs.writeFileSync(path.join(OUT,'results.json'),JSON.stringify({metadata,records,examples})+'\n');const keys=Object.keys(records[0]);fs.writeFileSync(path.join(OUT,'results.csv'),keys.join(',')+'\n'+records.map(r=>keys.map(k=>r[k]===null?'':r[k]).join(',')).join('\n')+'\n');console.log(`Saved ${records.length} payload trials.`);
}
module.exports={trial,loads,protocol};if(require.main===module)main();
