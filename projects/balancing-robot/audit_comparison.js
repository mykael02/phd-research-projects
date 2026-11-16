'use strict';
const fs=require('fs'),path=require('path'),physics=require('./model.js'),{trial}=require('./run_comparison.js');const OUT=path.join(__dirname,'results/comparison'),data=JSON.parse(fs.readFileSync(path.join(OUT,'test.json'),'utf8')),original=physics.rk4;
const condition=r=>({initial_tilt_deg:r.initial_tilt_deg,push_force_n:r.push_force_n,torque_limit_nm:r.torque_limit_nm});
function substeps(n){physics.rk4=(s,u,f,dt,p)=>{for(let i=0;i<n;i++)s=original(s,u,f,dt/n,p);return s;};}
try{const comparisons=[];substeps(2);for(const [index,r] of data.records.entries()){if(r.phase!=='ideal_test')continue;const a=trial(r.variant,condition(r),{mass_kg:r.mass_kg,height_m:r.height_m}).metrics;comparisons.push({index,original:r.outcome,refined:a.outcome,peak_delta_deg:a.peak_tilt_deg-r.peak_tilt_deg,position_delta_m:a.final_position_m-r.final_position_m});}
 const changed=comparisons.filter(r=>r.original!==r.refined);substeps(4);for(const x of changed){const r=data.records[x.index];x.four_substeps=trial(r.variant,condition(r),{mass_kg:r.mass_kg,height_m:r.height_m}).metrics.outcome;}
 fs.writeFileSync(path.join(OUT,'integration_audit.json'),JSON.stringify({control_dt_s:.005,trials:comparisons.length,changed_outcomes:changed.length,comparisons},null,2)+'\n');console.log(`${comparisons.length} ideal test trials audited; ${changed.length} changed labels.`);
}finally{physics.rk4=original;}
