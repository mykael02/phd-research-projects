'use strict';
const fs=require('fs'),path=require('path'),crypto=require('crypto'),config=require('./config.js'),Payload=require('./visualisation/payload.js'),LQR=require('./visualisation/payload-lqr.js'),{SensedRobot}=require('./visualisation/sensing.js');
const ROOT=__dirname,OUT=path.join(ROOT,'results/comparison'),protocol=JSON.parse(fs.readFileSync(path.join(ROOT,'comparison_protocol.json'),'utf8'));
function trial(variant,condition,load,profile={id:'ideal',mode:'ideal'},seed=5003,capture=false){
 const estimated=LQR.estimate(variant,load),loaded=Payload.configure(config,load.mass_kg,load.height_m).config,c={...loaded,lqr_gain:LQR.gain(estimated.mass_kg,estimated.height_m)},r=new SensedRobot('lqr',c,condition.initial_tilt_deg,condition.torque_limit_nm,{...profile,seed,accelSigma:50});
 let lastBad=null,tail=[0,0,0],n=0;const rows=[],reference=condition.push_force_n===0?0:2.2;
 function record(){const s=r.state,t=r.time;if(t>=reference-1e-10&&(Math.abs(s[0])>.03||Math.abs(s[1])>.03||Math.abs(s[2])>Math.PI/180))lastBad=t;if(t>=9-1e-10){for(let i=0;i<3;i++)tail[i]+=s[i]**2;n++;}if(capture)rows.push({time:t,state:s.slice(),estimate:r.estimate.slice(),torque:r.torque});}
 record();while(r.time<10-1e-10&&!r.fallen){r.step(r.time>=2-1e-10&&r.time<2.2-1e-10?condition.push_force_n:0);record();}
 const settled=Math.max(reference,lastBad===null?reference:lastBad+c.dt),recovery=!r.fallen&&r.time-settled>=1-1e-10?settled-reference:null,full=r.time>=10-1e-10&&!r.fallen;
 return {metrics:{variant,profile:profile.id,seed,...load,estimated_mass_kg:estimated.mass_kg,estimated_height_m:estimated.height_m,...condition,outcome:r.fallen?'fallen':recovery===null?'unsettled':'recovered',duration_s:r.time,recovery_s:recovery,peak_tilt_deg:r.peak,max_displacement_m:r.maxX,final_position_m:r.state[0],torque_squared_integral:r.effort,saturation_s:r.saturatedSteps*c.dt,tail_tilt_rms_deg:full?Math.sqrt(tail[2]/n)*180/Math.PI:null,tail_position_rms_m:full?Math.sqrt(tail[0]/n):null,tail_speed_rms_m_s:full?Math.sqrt(tail[1]/n):null},rows};
}
function main(){fs.mkdirSync(OUT,{recursive:true});const phase=process.argv.includes('--development')?'development':'test';const files=['comparison_protocol.json','config.js','model.js','visualisation/sensing.js','visualisation/payload.js','visualisation/payload-lqr.js','visualisation/payload-gains.js','design_payload_lqr.py','run_comparison.js'];const metadata={protocol,node:process.version,phase,source_sha256:Object.fromEntries(files.map(f=>[f,crypto.createHash('sha256').update(fs.readFileSync(path.join(ROOT,f))).digest('hex')]))};
 fs.writeFileSync(path.join(OUT,phase+'_snapshot.json'),JSON.stringify(metadata,null,2)+'\n');const records=[],examples={};
 if(phase==='development'){for(const load of protocol.development_loads)for(const condition of protocol.development_conditions)for(const variant of ['fixed','exact'])records.push({phase,...trial(variant,condition,load).metrics});}
 else{for(const m of protocol.test_masses_kg)for(const h of protocol.test_heights_m){const load={mass_kg:m,height_m:h};
  for(const t of protocol.ideal_test.tilts_deg)for(const f of protocol.ideal_test.forces_n)for(const limit of protocol.ideal_test.torque_limits_nm)for(const v of protocol.variants){const condition={initial_tilt_deg:t,push_force_n:f,torque_limit_nm:limit},e=protocol.example,save=m===e.mass_kg&&h===e.height_m&&t===e.initial_tilt_deg&&f===e.push_force_n&&limit===e.torque_limit_nm;const result=trial(v,condition,load,undefined,5003,save);records.push({phase:'ideal_test',...result.metrics});if(save)examples[v]=result;}
  for(const scenario of protocol.sensing_test)for(const profile of protocol.profiles)for(let i=0;i<protocol.repetitions;i++)for(const v of protocol.variants){const {id,...condition}=scenario;records.push({phase:'sensing_test',scenario:id,...trial(v,condition,load,profile,5003+104729*i).metrics});}
  console.log(`Test load ${m} kg at ${h} m complete; ${records.length} records`);
 }}
 fs.writeFileSync(path.join(OUT,phase+'.json'),JSON.stringify({metadata,records,examples})+'\n');const keys=[...new Set(records.flatMap(Object.keys))];fs.writeFileSync(path.join(OUT,phase+'.csv'),keys.join(',')+'\n'+records.map(r=>keys.map(k=>r[k]??'').join(',')).join('\n')+'\n');console.log(`Saved ${records.length} ${phase} records.`);
}
module.exports={trial,protocol};if(require.main===module)main();
