/* Logic-only UI smoke test. This does not replace visual inspection in a browser. */
const fs=require('node:fs'),vm=require('node:vm'),path=require('node:path'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..'),html=fs.readFileSync(path.join(root,'index.html'),'utf8');
const elements={};
const context=new Proxy({}, {get(target,key){if(key in target)return target[key];return (...args)=>{for(const arg of args)if(typeof arg==='number')assert(Number.isFinite(arg),'Non-finite canvas coordinate');};},set(target,key,value){target[key]=value;return true;}});
for(const match of html.matchAll(/id="([^"]+)"/g))elements[match[1]]={value:'',textContent:'',disabled:false,width:0,height:0,getBoundingClientRect:()=>({width:500,height:240}),getContext:()=>context};
elements.tilt.value='5';elements.force.value='3';elements.limit.value='.7';
let frameCallback,stamp=0;
const doc={hidden:false,getElementById(id){assert(elements[id],`Missing element ${id}`);return elements[id];},addEventListener(){}};
const sandbox={document:doc,window:{location:{search:''},devicePixelRatio:1,addEventListener(){}},URLSearchParams,requestAnimationFrame:cb=>{frameCallback=cb;},console};
vm.createContext(sandbox);
for(const file of ['config.js','model.js','app.js'])vm.runInContext(fs.readFileSync(path.join(root,file),'utf8'),sandbox,{filename:file});
function frames(count){for(let i=0;i<count;i++){stamp+=1000/60;frameCallback(stamp);}}
assert(elements['pid-angle'].textContent==='5.00°');
elements['push-right'].onclick();assert(elements.message.textContent.includes('before applying'));
elements.run.onclick();frames(120);assert(elements.clock.textContent!=='0.00 / 20.00 s');
elements['push-right'].onclick();assert(elements.message.textContent.includes('3 N push right'));
frames(30);assert(Math.abs(parseFloat(elements['pid-angle'].textContent))>.1);
elements.run.onclick();const paused=elements.clock.textContent;frames(30);assert.equal(elements.clock.textContent,paused);
elements.run.onclick();frames(1200);assert(elements.run.disabled);assert.equal(elements.clock.textContent,'20.00 / 20.00 s');
elements.reset.onclick();assert.equal(elements.clock.textContent,'0.00 / 20.00 s');assert(!elements.run.disabled);
elements.force.value='30';elements.force.oninput();elements.run.onclick();frames(120);elements['push-left'].onclick();frames(120);
assert(elements['pid-status'].textContent.includes('Fallen'));assert(elements['lqr-status'].textContent.includes('Fallen'));
elements.tilt.value='-10';elements.tilt.oninput();assert.equal(elements['lqr-angle'].textContent,'-10.00°');
elements.limit.value='.1';elements.limit.oninput();assert.equal(elements['limit-value'].textContent,'0.10 N·m');
console.log('UI logic passed: initial rendering, run, both push directions, pause/resume, end, falls, reset, tilt and torque controls.');
