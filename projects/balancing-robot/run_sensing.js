/* Shared browser/Node sensing study. No controller access to evaluation truth. */
'use strict';
const fs=require('fs'),path=require('path'),crypto=require('crypto');
const {SensedRobot}=require('./visualisation/sensing.js'),config=require('./config.js');
const ROOT=__dirname,protocol=JSON.parse(fs.readFileSync(path.join(ROOT,'sensing_protocol.json'),'utf8')),OUT=path.join(ROOT,'results','sensing');
function trial(controller,scenario,options,duration=10,saveRows=false){
 const robot=new SensedRobot(controller,config,scenario.initial_tilt_deg,scenario.torque_limit_nm,options),rows=[],truth=[];
 let lastBad=null,errorSq=0,sourceErrorSq=0,measurementSq=0,n=0;
 function capture(){const s=robot.state,r=robot.readout,t=robot.time;truth.push(s.slice());const old=truth[r.sourceTick];
  errorSq+=(r.estimate[2]-s[2])**2;sourceErrorSq+=(r.estimate[2]-old[2])**2;measurementSq+=(r.measurement[2]-s[2])**2;n++;
  if(t>=2.2-1e-10&&(Math.abs(s[0])>.03||Math.abs(s[1])>.03||Math.abs(s[2])>Math.PI/180))lastBad=t;
  if(saveRows)rows.push({time:t,state:s.slice(),measurement:r.measurement.slice(),estimate:r.estimate.slice(),sourceTime:r.sourceTime,sourceTick:r.sourceTick,torque:robot.torque});
 }
 capture();while(robot.time<duration-1e-10&&!robot.fallen){const force=robot.time>=2-1e-10&&robot.time<2.2-1e-10?scenario.push_force_n:0;robot.step(force);capture();}
 const settled=Math.max(2.2,lastBad===null?2.2:lastBad+config.dt),recovery=!robot.fallen&&robot.time-settled>=1-1e-10?settled-2.2:null;
 const metrics={scenario:scenario.id,controller,observer:options.mode,angleNoiseDeg:robot.sensor.options.angleNoiseDeg,gyroNoiseDegS:robot.sensor.options.gyroNoiseDegS,delayMs:robot.sensor.options.delayMs,seed:options.seed,
  fallen:robot.fallen,outcome:robot.fallen?'fallen':recovery===null?'unsettled':'recovered',duration_s:robot.time,recovery_s:recovery,peak_tilt_deg:robot.peak,max_displacement_m:robot.maxX,final_position_m:robot.state[0],torque_squared_integral:robot.effort,saturation_s:robot.saturatedSteps*config.dt,
  estimate_current_rmse_deg:Math.sqrt(errorSq/n)*180/Math.PI,estimate_source_rmse_deg:Math.sqrt(sourceErrorSq/n)*180/Math.PI,measurement_current_rmse_deg:Math.sqrt(measurementSq/n)*180/Math.PI,samples:n};
 return {metrics,rows};
}
function design(){const cells=new Map();function add(a,g,d,family){const key=[a,g,d].join('/');if(!cells.has(key))cells.set(key,{angleNoiseDeg:a,gyroNoiseDegS:g,delayMs:d,families:[]});if(!cells.get(key).families.includes(family))cells.get(key).families.push(family);}
 for(const a of protocol.angle_only_sigma_deg)add(a,0,0,'angle_only');for(const g of protocol.gyro_only_sigma_deg_s)add(0,g,0,'gyro_only');for(const a of protocol.combined_angle_sigma_deg)for(const d of protocol.delay_ms)add(a,a*protocol.combined_gyro_multiplier,d,'combined');return [...cells.values()];}
function main(){fs.mkdirSync(OUT,{recursive:true});const records=[],examples={},baselines=[],cells=design();
 const hash=file=>crypto.createHash('sha256').update(fs.readFileSync(path.join(ROOT,file))).digest('hex');
 const provenance={protocol,node:process.version,control_step_s:config.dt,config,source_sha256:Object.fromEntries(['model.js','config.js','visualisation/sensing.js','run_sensing.js','sensing_protocol.json'].map(f=>[f,hash(f)]))};
 fs.writeFileSync(path.join(OUT,'protocol_snapshot.json'),JSON.stringify(provenance,null,2)+'\n');
 for(const scenario of protocol.scenarios)for(const controller of protocol.controllers){
  baselines.push(trial(controller,scenario,{mode:'ideal',seed:1001},10).metrics);
  for(const mode of protocol.observers)for(const cell of cells){const repetitions=cell.angleNoiseDeg||cell.gyroNoiseDegS?protocol.stochastic_repetitions:1;
   for(let i=0;i<repetitions;i++){const seed=1001+104729*i,options={...cell,mode,seed,accelSigma:protocol.kalman_accel_sigma_rad_s2};const save=scenario.id===protocol.example.scenario&&cell.angleNoiseDeg===protocol.example.angleNoiseDeg&&cell.gyroNoiseDegS===protocol.example.gyroNoiseDegS&&cell.delayMs===0&&i===0;
    const result=trial(controller,scenario,options,10,save);records.push({...result.metrics,families:cell.families});if(save)examples[controller+'_'+mode]=result;
   }
  }
  console.log(`${scenario.id} ${controller}: ${records.length} sensing trials`);
 }
 const payload={metadata:provenance,baselines,records,examples};fs.writeFileSync(path.join(OUT,'results.json'),JSON.stringify(payload)+'\n');
 const fields=Object.keys(records[0]).filter(k=>k!=='families');fs.writeFileSync(path.join(OUT,'results.csv'),fields.join(',')+'\n'+records.map(r=>fields.map(k=>r[k]===null?'':r[k]).join(',')).join('\n')+'\n');
 console.log(`Saved ${records.length} unique trials and ${baselines.length} ideal baselines.`);
}
module.exports={trial,design};if(require.main===module)main();
