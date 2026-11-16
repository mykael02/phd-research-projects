/* Live 200 Hz planar physics and archived trajectories share one 3D renderer. */
(function(){'use strict';
const $=id=>document.getElementById(id),data=window.REPLAY_DATA,C=window.ReplayCore;
let choice=0,time=0,playing=false,last=0,scene,camera,renderer,robots=[],radius=1.45,azimuth=.82,elevation=.52,targetX=0;
const R=.045,L=.18;
let mode='live',live,accumulator=0;
const isLive=()=>mode==='live';
data.scenarios.forEach((s,i)=>{const option=document.createElement('option');option.value=i;option.textContent=s.label;$('scenario').appendChild(option);});
function material(color,extra={}){return new THREE.MeshStandardMaterial({color,roughness:.58,metalness:.2,...extra});}
function mesh(geometry,mat,parent,x=0,y=0,z=0){const m=new THREE.Mesh(geometry,mat);m.position.set(x,y,z);m.castShadow=true;m.receiveShadow=true;parent.add(m);return m;}
function label(text,color){const c=document.createElement('canvas');c.width=384;c.height=80;const g=c.getContext('2d');g.fillStyle='rgba(255,255,255,.92)';g.fillRect(0,0,384,80);g.fillStyle=color;g.font='600 24px Arial';g.textAlign='center';g.fillText(text,192,50,360);const s=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(c),depthTest:false}));s.scale.set(.28,.0583,1);return s;}
function robot(name,color,lane){const root=new THREE.Group();root.position.z=lane;scene.add(root);const pitch=new THREE.Group();root.add(pitch);
 const dark=material(0x243744),metal=material(0xb6c5cd),paint=material(color),yellow=material(0xf6c247);
 mesh(new THREE.CylinderGeometry(.012,.012,.22,20),metal,root).rotation.x=Math.PI/2;
 const wheels=[];for(const z of [-.115,.115]){const w=new THREE.Group();w.position.z=z;root.add(w);mesh(new THREE.CylinderGeometry(R,R,.035,40),dark,w).rotation.x=Math.PI/2;mesh(new THREE.CylinderGeometry(.027,.027,.038,32),metal,w).rotation.x=Math.PI/2;mesh(new THREE.BoxGeometry(.055,.006,.04),paint,w);wheels.push(w);}
 mesh(new THREE.BoxGeometry(.025,.18,.025),metal,pitch,0,.09,-.065);mesh(new THREE.BoxGeometry(.025,.18,.025),metal,pitch,0,.09,.065);
 mesh(new THREE.BoxGeometry(.085,.20,.145),paint,pitch,0,L,0);mesh(new THREE.BoxGeometry(.006,.09,.105),dark,pitch,.045,L,0);
 const com=mesh(new THREE.SphereGeometry(.008,24,16),new THREE.MeshBasicMaterial({color:0xf6c247,depthTest:false}),pitch,0,L,0);com.renderOrder=10;com.castShadow=false; // True COM projection, visible through the schematic shell.
 const tag=label(name,name==='PID-type'?'#a64311':'#086c77');root.add(tag);tag.position.set(0,.49,0);
 const arrow=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(0,L,0),.18,0xba7900,.035,.02);scene.add(arrow);arrow.visible=false;
 const estimated=new THREE.Group();scene.add(estimated);const outline=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(.09,.205,.15)),new THREE.LineBasicMaterial({color:0x9450c3,depthTest:false}));outline.position.y=L;outline.renderOrder=11;estimated.add(outline);estimated.visible=false;
 const payload=mesh(new THREE.BoxGeometry(Payload.shape.width,Payload.shape.height,Payload.shape.depth),material(0xd5a436,{transparent:true,opacity:.82,depthTest:false}),pitch,0,L,0);payload.name='payload-body';payload.renderOrder=6;payload.visible=false;
 const payloadOutline=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(Payload.shape.width,Payload.shape.height,Payload.shape.depth)),new THREE.LineBasicMaterial({color:0x9450c3,depthTest:false}));estimated.add(payloadOutline);payloadOutline.renderOrder=12;
 const payloadSupport=mesh(new THREE.BoxGeometry(.008,1,.008),metal,pitch);payloadSupport.visible=false;
 return {root,pitch,wheels,arrow,lane,estimated,com,payload,payloadOutline,payloadSupport,tag};}
function setup(){scene=new THREE.Scene();scene.background=new THREE.Color(0xe6ecef);camera=new THREE.PerspectiveCamera(38,1,.01,30);renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;$('scene').appendChild(renderer.domElement);
 scene.add(new THREE.HemisphereLight(0xffffff,0x7c8b93,2));const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(-1,3,2);light.castShadow=true;light.shadow.mapSize.set(2048,2048);scene.add(light);
 const floor=mesh(new THREE.PlaneGeometry(20,20),material(0xe6ecef),scene,0,-.002,0);floor.rotation.x=-Math.PI/2;floor.castShadow=false;const grid=new THREE.GridHelper(10,100,0xa4b5c1,0xc8d3db);grid.position.y=.0002;scene.add(grid);
 for(const z of [-.3,.3]){const origin=mesh(new THREE.BoxGeometry(.004,.001,.28),material(0x738ca0),scene,0,.001,z);origin.castShadow=false;}
 robots=[robot('PID-type',0xbd531e,-.3),robot('LQR',0x087b88,.3)];
 const canvas=renderer.domElement;let drag=null;canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});canvas.addEventListener('pointermove',e=>{if(!drag)return;azimuth-=(e.clientX-drag[0])*.008;elevation=Math.max(.04,Math.min(1.45,elevation+(e.clientY-drag[1])*.006));drag=[e.clientX,e.clientY];});for(const type of ['pointerup','pointercancel'])canvas.addEventListener(type,()=>drag=null);canvas.addEventListener('wheel',e=>{e.preventDefault();radius=Math.max(.65,Math.min(8,radius*Math.exp(e.deltaY*.001)));},{passive:false});
 new ResizeObserver(()=>{const box=$('scene').getBoundingClientRect();renderer.setSize(box.width,box.height);camera.aspect=box.width/box.height;camera.updateProjectionMatrix();}).observe($('scene'));
}
function update(){const scenario=data.scenarios[choice],inLive=isLive();if(inLive)time=live.time;
 const duration=inLive?20:10;$('time').value=time;$('time-label').textContent=`${time.toFixed(2)} / ${duration.toFixed(2)} s`;
 $('condition').textContent=inLive?`Live physics at 200 Hz \u00b7 initial tilt ${$('tilt').value}\u00b0 \u00b7 torque limit ${Number($('limit').value).toFixed(2)} N\u00b7m \u00b7 camera follows mean displacement.`:`Initial tilt ${scenario.condition[0]}\u00b0 \u00b7 signed push ${scenario.condition[1]} N from 2.00 to 2.20 s \u00b7 total motor torque limit ${scenario.condition[2].toFixed(2)} N\u00b7m. Camera follows mean displacement.`;
 let mean=0,pushing=false;
 ['pid','lqr'].forEach((name,i)=>{const trial=scenario[name],sim=inLive?live.robots[i]:null,row=inLive?{state:sim.state,torque:sim.fallen?0:sim.torque}:C.sample(trial.rows,time),fallen=inLive?sim.fallen:trial.metrics.fallen&&time>=trial.rows.at(-1).time;mean+=row.state[0]/2;
 $(name+'-angle').textContent=(row.state[2]*180/Math.PI).toFixed(2)+'\u00b0';$(name+'-x').textContent=row.state[0].toFixed(3)+' m';$(name+'-torque').textContent=row.torque.toFixed(3)+' N\u00b7m';$(name+'-state').textContent=fallen?(inLive?'Fallen \u00b7 stopped':'Fallen \u00b7 replay frozen'):inLive?(time===0?'Ready':live.complete?'Run complete':playing?'Live':'Paused'):time>=10?'Final: '+trial.outcome:'Recorded state';
 const force=fallen?0:inLive?live.force:C.forceAt(time,scenario.condition[1],trial.rows.at(-1).time,trial.metrics.fallen);pushing=pushing||force!==0;
 if(renderer){const r=robots[i];r.root.position.set(row.state[0],R,r.lane);r.pitch.rotation.z=-row.state[2];r.wheels.forEach(w=>w.rotation.z=-row.state[0]/R);const bodyL=inLive?live.config.parameters.length:L;r.com.position.y=bodyL;r.payload.visible=inLive&&live.payload.mass_kg>0;r.payload.position.y=inLive?live.payload.height_m:L;r.payloadSupport.visible=r.payload.visible;r.payloadSupport.position.y=(L+r.payload.position.y)/2;r.payloadSupport.scale.y=Math.abs(r.payload.position.y-L);r.payloadOutline.visible=r.payload.visible;r.payloadOutline.position.y=r.payload.position.y;const centre=C.bodyCentre(row.state,R,bodyL);r.arrow.position.set(centre[0],centre[1],r.lane);r.arrow.setDirection(new THREE.Vector3(Math.sign(force)||1,0,0));r.arrow.visible=force!==0;r.estimated.visible=inLive&&live.options.mode!=='ideal'&&$('show-estimate').checked;if(inLive){r.estimated.position.set(sim.estimate[0],R,r.lane);r.estimated.rotation.z=-sim.estimate[2];}}
 });
 targetX=mean;$('pulse').textContent=inLive?(pushing?`${playing?'Force pulse':'Paused pulse'}: ${live.force} N`:'No external force'):pushing?`Force pulse: ${scenario.condition[1]} N`:time<2?'Push scheduled at 2.00 s':time<2.2?'No force applied':'Force pulse ended';$('pulse').classList.toggle('active',pushing);
 if(renderer){camera.position.set(targetX+radius*Math.cos(elevation)*Math.sin(azimuth),.13+radius*Math.sin(elevation),radius*Math.cos(elevation)*Math.cos(azimuth));camera.lookAt(targetX,.13,0);renderer.render(scene,camera);}
 if(inLive)SensingUI.render(live,!playing);
}
function playState(){ $('play').disabled=isLive()&&live.complete;$('play').textContent=isLive()?(live.complete?'Run complete':playing?'Pause':live.time?'Resume':'Run simulation'):playing?'Pause replay':'Play replay';}
function reset(){let next;try{if(isLive())next=new LiveSession(BALANCE_CONFIG,Number($('tilt').value),Number($('limit').value),SensingUI.options(),{mass_kg:Number($('payload-mass').value),height_m:Number($('payload-height').value)},{mode:$('comparison').value,useActual:$('use-actual').checked,estimatedMass:Number($('estimated-mass').value),estimatedHeight:Number($('estimated-height').value)});}catch(error){$('message').textContent=error.message;return;}playing=false;time=0;last=0;accumulator=0;if(isLive())live=next;
 const names=isLive()?live.labels:['PID-type','LQR'];['pid','lqr'].forEach((id,i)=>{$(id+'-label').textContent=names[i];$(id+'-trace-title').textContent=names[i];$(id+'-trace').setAttribute?.('aria-label',names[i]+' true, measured and estimated state over time');if(renderer){const map=robots[i].tag.material.map,g=map.image.getContext('2d');g.fillStyle='rgba(255,255,255,.92)';g.fillRect(0,0,384,80);g.fillStyle=i?'#086c77':'#a64311';g.font='600 24px Arial';g.textAlign='center';g.fillText(names[i],192,50,360);map.needsUpdate=true;}});
 $('comparison-controls').hidden=!isLive();$('estimate-controls').hidden=$('comparison').value!=='payload_lqr';
 if(isLive()){const automatic=$('use-actual').checked;if(automatic){$('estimated-mass').value=live.payload.mass_kg;$('estimated-height').value=live.payload.height_m;}for(const id of ['estimated-mass','estimated-height'])$(id).disabled=automatic;$('estimated-mass-value').textContent=Number($('estimated-mass').value).toFixed(2)+' kg';$('estimated-height-value').textContent=Number($('estimated-height').value).toFixed(2)+' m';$('comparison-summary').textContent=live.comparison.mode==='payload_lqr'?'Orange: fixed nominal gains. Teal: gains designed for '+live.comparison.estimated_payload.mass_kg.toFixed(2)+' kg at '+live.comparison.estimated_payload.height_m.toFixed(2)+' m. Same state and effort weights.':'Original PID-type and fixed nominal LQR.';}
 $('live-controls').hidden=!isLive();$('scenario-control').hidden=isLive();$('time').disabled=isLive();$('time').max=isLive()?20:10;$('time-title').textContent=isLive()?'Simulation time':'Replay time';$('restart').textContent=isLive()?'Reset':'Restart';
 $('tilt-value').textContent=$('tilt').value+'\u00b0';$('limit-value').textContent=Number($('limit').value).toFixed(2)+' N\u00b7m';$('force-value').textContent=$('force').value+' N \u00b7 0.2 s';
 $('payload-controls').hidden=!isLive();if(isLive()){$('payload-mass-value').textContent=live.payload.mass_kg.toFixed(2)+' kg';$('payload-height-value').textContent=live.payload.height_m.toFixed(2)+' m';const p=live.config.parameters;$('payload-summary').textContent='Loaded body: '+p.body_mass.toFixed(2)+' kg | COM: '+p.length.toFixed(3)+' m from axle | COM inertia: '+p.body_inertia.toFixed(5)+' kg m^2. Wheel/base mass is additional.';}
 $('sensor-controls').hidden=!isLive();$('sensing-traces').hidden=!isLive();SensingUI.labels();
 $('message').textContent=isLive()?'Ready. Run, then push both robots.':'Recorded experiment. Choose Live simulation to apply your own forces.';playState();update();}
$('mode').onchange=()=>{mode=$('mode').value;reset();};
$('play').onclick=()=>{if(isLive()&&live.complete)return;if(!isLive()&&time>=10)time=0;playing=!playing;last=0;accumulator=0;playState();$('message').textContent=playing?(isLive()?'Running. Apply a push to both robots.':'Playing recorded experiment.'):'Paused.';update();};
$('restart').onclick=reset;$('scenario').onchange=()=>{choice=Number($('scenario').value);reset();};
$('time').oninput=()=>{if(isLive())return;playing=false;time=Number($('time').value);playState();update();};
function push(direction){if(!isLive())return;if(!playing||live.complete){$('message').textContent='Run or resume the simulation before applying a push.';return;}const force=direction*Number($('force').value);live.push(force);$('message').textContent=`${Math.abs(force)} N push ${direction>0?'right (+X)':'left (-X)'} for 0.2 simulated seconds.`;update();}
$('push-left').onclick=()=>push(-1);$('push-right').onclick=()=>push(1);$('tilt').oninput=reset;$('limit').oninput=reset;$('force').oninput=()=>{$('force-value').textContent=$('force').value+' N \u00b7 0.2 s';};
for(const id of ['comparison','use-actual'])$(id).onchange=reset;
for(const id of ['estimated-mass','estimated-height'])$(id).oninput=reset;
for(const id of ['payload-mass','payload-height'])$(id).oninput=reset;
for(const id of ['observer','angle-noise','gyro-noise','delay','seed'])$(id).onchange=reset;
$('trace-state').onchange=()=>{if(isLive())SensingUI.render(live,true);};$('show-estimate').onchange=update;
$('export-run').onclick=()=>{if(!isLive())return;const payload={settings:live.options,initial_tilt_deg:live.initialTilt,torque_limit_nm:live.torqueLimit,config:{parameters:live.config.parameters,dt:live.dt,fall_angle_degrees:BALANCE_CONFIG.fall_angle_degrees},nominal_config:BALANCE_CONFIG,payload:live.payload,comparison:live.comparison,controller_labels:live.labels,controller_gains:live.controllerGains,events:live.events,history:live.history};const a=document.createElement('a'),url=URL.createObjectURL(new Blob([JSON.stringify(payload)],{type:'application/json'}));a.href=url;a.download='balancing-payload-sensing-run.json';a.click();URL.revokeObjectURL(url);};
$('view-orbit').onclick=()=>{azimuth=.82;elevation=.52;radius=1.45;};$('view-side').onclick=()=>{azimuth=0;elevation=.12;radius=1.45;};$('view-front').onclick=()=>{azimuth=Math.PI/2;elevation=.2;radius=1.45;};
$('snapshot').onclick=()=>{if(!renderer)return;update();const a=document.createElement('a');a.download=`balance-3d-${isLive()?'live':'case-'+(choice+1)}-t${time.toFixed(2)}.png`;a.href=renderer.domElement.toDataURL('image/png');a.click();};
try{setup();}catch(error){console.error(error);renderer=null;$('render-error').hidden=false;$('snapshot').disabled=true;}
function frame(now){if(playing&&last){const elapsed=Math.min((now-last)/1000,.1)*Number($('speed').value);
 if(isLive()){accumulator+=elapsed;while(accumulator>=live.dt&&!live.complete){live.step();accumulator-=live.dt;}if(live.complete){playing=false;playState();$('message').textContent=live.robots.every(r=>r.fallen)?'Both robots fell. Reset to try again.':'20-second run complete. Reset to try another run.';}}
 else{time=Math.min(10,time+elapsed);if(time===10){playing=false;playState();}}
 }last=now;update();requestAnimationFrame(frame);}
 document.addEventListener('visibilitychange',()=>{last=0;accumulator=0;if(document.hidden&&playing){playing=false;playState();$('message').textContent='Paused while this tab is hidden.';update();}});
 reset();requestAnimationFrame(frame);
})();
