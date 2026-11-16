/* Runs UI logic with DOM/canvas mocks; no claim of visual browser verification. */
const fs=require('node:fs'),vm=require('node:vm'),path=require('node:path'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..'),report=fs.readFileSync(path.join(root,'results/robustness/report.html'),'utf8');
const data=JSON.parse(report.match(/<script id="study-data" type="application\/json">([\s\S]*?)<\/script>/)[1]);
const elements={};let buttons=[];
for(const [,id] of report.matchAll(/id="([^"]+)"/g)){
  elements[id]={value:'',textContent:'',href:'',listeners:{},addEventListener(k,v){this.listeners[k]=v;},set innerHTML(html){this.html=html;
    if(id.endsWith('-map')){buttons=buttons.filter(b=>b.dataset.controller!==id.slice(0,3));
      for(const match of html.matchAll(/data-controller="([^"]+)" data-tilt="([^"]+)" data-push="([^"]+)"/g))buttons.push({dataset:{controller:match[1],tilt:match[2],push:match[3]},setAttribute(k,v){this[k]=v;}});
    }},get innerHTML(){return this.html||'';}};
}
elements['study-data'].textContent=JSON.stringify(data);
const document={getElementById(id){assert(elements[id],id);return elements[id];},querySelectorAll(){return buttons;}};
const source=report.match(/<\/script><script>([\s\S]*?)<\/script>/)[1];
vm.runInNewContext(source,{document,URLSearchParams,console});
assert.equal(buttons.length,234);assert(elements.selection.textContent.startsWith('LQR'));
elements.torque.value='.1';elements.torque.listeners.change();
const chosen=buttons.find(b=>b.dataset.controller==='pid'&&b.dataset.tilt==='20'&&b.dataset.push==='30');
elements['pid-map'].listeners.click({target:{closest:()=>chosen}});
assert(elements.selection.textContent.includes('Fallen'));assert(elements['detail-context'].textContent.includes('before the push'));
assert(elements.replay.href.includes('force=30'));assert(elements.replay.href.includes('limit=0.1'));
assert.equal(buttons.filter(b=>b['aria-pressed']==='true').length,1);

const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
function replay(tilt,push,limit){
  const els={},ctx=new Proxy({}, {get:(o,k)=>k in o?o[k]:(()=>{}),set:(o,k,v)=>(o[k]=v,true)});
  for(const [,id] of html.matchAll(/id="([^"]+)"/g))els[id]={value:'',textContent:'',width:0,height:0,getBoundingClientRect:()=>({width:500,height:240}),getContext:()=>ctx};
  els.tilt.value='5';els.force.value='3';els.limit.value='.7';let callback,stamp=0;
  const sandbox={document:{hidden:false,getElementById(id){assert(els[id],id);return els[id];},addEventListener(){}},window:{location:{search:`?scheduled=1&tilt=${tilt}&force=${push}&limit=${limit}&duration=10`},devicePixelRatio:1,addEventListener(){}},URLSearchParams,requestAnimationFrame:cb=>{callback=cb;},console};
  vm.createContext(sandbox);for(const f of ['config.js','model.js','app.js'])vm.runInContext(fs.readFileSync(path.join(root,f),'utf8'),sandbox);
  assert(els.message.textContent.includes('Study replay'));els.run.onclick();
  for(let i=0;i<700;i++){stamp+=1000/60;callback(stamp);}
  for(const controller of ['pid','lqr']){
    const row=data.records.find(r=>r.controller===controller&&r.initial_tilt_deg===tilt&&r.push_force_n===push&&r.torque_limit_nm===limit);
    assert(Math.abs(parseFloat(els[controller+'-angle'].textContent)-row.final_tilt_deg)<=.005001);
    assert(Math.abs(parseFloat(els[controller+'-position'].textContent)-row.final_position_m)<=.000501);
    assert.equal(els[controller+'-status'].textContent.includes('Fallen'),row.fallen);
  }
  assert(els.run.disabled);
}
for(const values of [[5,5,.7],[-5,-5,.7],[0,0,.7],[20,30,.1],[0,30,1]])replay(...values);
console.log('Study controls and selection passed; five scheduled replays match saved Python outcomes and final telemetry for both controllers.');
