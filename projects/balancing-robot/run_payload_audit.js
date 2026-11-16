'use strict';
// Separate process: change only RK4 substeps; controller and sensor remain at 200 Hz.
const fs=require('fs'),path=require('path'),physics=require('./model.js'),{trial}=require('./run_payload.js');
const OUT=path.join(__dirname,'results/payload'),data=JSON.parse(fs.readFileSync(path.join(OUT,'results.json'),'utf8')),original=physics.rk4;
function setSubsteps(n){physics.rk4=(s,u,f,dt,p)=>{for(let i=0;i<n;i++)s=original(s,u,f,dt/n,p);return s;};}
function run(){const comparisons=[];setSubsteps(2);for(const [index,r] of data.records.entries()){if(r.phase!=='ideal_grid')continue;
 const refined=trial(r.controller,r,{mass_kg:r.mass_kg,height_m:r.height_m}).metrics;
 comparisons.push({index,original:r.outcome,refined:refined.outcome,peak_tilt_delta_deg:refined.peak_tilt_deg-r.peak_tilt_deg,final_position_delta_m:refined.final_position_m-r.final_position_m,duration_delta_s:refined.duration_s-r.duration_s});}
 const changed=comparisons.filter(c=>c.original!==c.refined);setSubsteps(4);for(const c of changed){const r=data.records[c.index];c.four_substeps=trial(r.controller,r,{mass_kg:r.mass_kg,height_m:r.height_m}).metrics.outcome;}
 fs.writeFileSync(path.join(OUT,'integration_audit.json'),JSON.stringify({control_dt_s:.005,original_substeps:1,refined_substeps:2,trials:comparisons.length,changed_outcomes:changed.length,comparisons},null,2)+'\n');console.log(`Integration audit: ${comparisons.length} deterministic trials; ${changed.length} outcome changes.`);
}
try{run();}finally{physics.rk4=original;}
