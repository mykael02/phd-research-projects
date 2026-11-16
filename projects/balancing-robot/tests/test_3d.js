'use strict';
const assert=require('assert'),fs=require('fs'),path=require('path'),vm=require('vm');
const root=path.resolve(__dirname,'..'),core=require('../visualisation/replay-core.js');
const context={window:{}};vm.createContext(context);for(const file of ['replay-data.js','experiment-data.js'])vm.runInContext(fs.readFileSync(path.join(root,'visualisation',file),'utf8'),context);
const data=context.window.REPLAY_DATA,records=context.window.EXPERIMENT_DATA,analysis=JSON.parse(fs.readFileSync(path.join(root,'publication/data/analysis.json')));
assert.equal(records.length,1638);assert.equal(data.scenarios.length,3);
for(const scenario of data.scenarios)for(const name of ['pid','lqr']){
 const trial=scenario[name],source=Object.values(analysis.examples).find(e=>e.controller===name&&String(e.condition)===String(scenario.condition));assert.equal(JSON.stringify(trial.rows),JSON.stringify(source.rows));assert.equal(core.sample(trial.rows,0).time,0);assert.equal(core.sample(trial.rows,10).time,trial.rows.at(-1).time);for(let i=1;i<trial.rows.length;i++)assert(trial.rows[i].time>trial.rows[i-1].time);assert.equal(core.sample(trial.rows,.007).time,.005);
}
const centre=core.bodyCentre([1,0,Math.PI/6,0],.045,.18);assert(Math.abs(centre[0]-1.09)<1e-12);assert(Math.abs(centre[1]-(.045+.18*Math.sqrt(3)/2))<1e-12);
assert.equal(core.forceAt(2.05,5,.85,true),0);assert.equal(core.forceAt(2.05,5,10,false),5);assert.equal(core.forceAt(2.2,5,10,false),0);
for(const [name,counts] of [['pid',[215,220,384]],['lqr',[357,0,462]]])['recovered','unsettled','fallen'].forEach((o,i)=>assert.equal(records.filter(r=>r.controller===name&&r.outcome===o).length,counts[i]));
console.log('3D data fidelity, sampling, COM geometry, force timing and outcome-count checks passed.');
