/* All controls run locally. No server, external assets, or background requests. */
(() => {
  'use strict';
  const $=id=>document.getElementById(id), cfg=BALANCE_CONFIG, colors={pid:'#f0b06b',lqr:'#71d7c7'};
  const query=new URLSearchParams(window.location.search);
  function parameter(name,fallback,min,max){const raw=query.get(name),n=raw===null?NaN:Number(raw);return Number.isFinite(n)&&n>=min&&n<=max?n:fallback;}
  const scheduled=query.get('scheduled')==='1', scheduledDirection=parameter('force',3,-30,30)<0?-1:1;
  if(scheduled){$('tilt').value=parameter('tilt',5,-20,20);$('limit').value=parameter('limit',.7,.1,1);$('force').value=Math.abs(parameter('force',3,-30,30));}
  let robots, running=false, time=0, pushEnd=0, pushForce=0, history=[], events=[], accumulator=0, last=null, lastUi=-1, scheduledDone=false;
  const step=cfg.dt, duration=scheduled?10:20;
  function reset(){running=false;time=0;pushEnd=0;pushForce=0;accumulator=0;last=null;lastUi=-1;events=[];scheduledDone=false;
    robots=['pid','lqr'].map(name=>new Balance.Robot(name,cfg,Number($('tilt').value),Number($('limit').value)));
    history=[{time:0,angles:robots.map(r=>r.state[2]*180/Math.PI)}];
    $('tilt-value').textContent=$('tilt').value+'°';$('limit-value').textContent=Number($('limit').value).toFixed(2)+' N·m';$('force-value').textContent=$('force').value+' N · 0.2 s';
    $('run').textContent='Run simulation';$('run').disabled=false;$('message').textContent=scheduled?`Study replay: ${scheduledDirection*Number($('force').value)} N push at 2.0 s; 10-second run. Click Run.`:'Ready. Run, then push both robots.';render();}
  function announce(message){$('message').textContent=message;}
  $('run').onclick=()=>{if(time>=duration||robots.every(r=>r.fallen))return;running=!running;last=null;accumulator=0;$('run').textContent=running?'Pause':'Resume';announce(running?(scheduled?'Running study replay; scheduled push at 2.0 s.':'Running. Both robots receive the same push.'):'Paused.');};
  $('reset').onclick=reset;
  function push(direction){if(!running){announce('Run or resume the simulation before applying a push.');return;}pushForce=direction*Number($('force').value);pushEnd=time+.2;events.push({time,force:pushForce});announce(`${Math.abs(pushForce)} N push ${direction>0?'right':'left'} for 0.2 seconds.`);}
  $('push-left').onclick=()=>push(-1);$('push-right').onclick=()=>push(1);
  $('tilt').oninput=()=>{$('tilt-value').textContent=$('tilt').value+'°';reset();};
  $('limit').oninput=()=>{$('limit-value').textContent=Number($('limit').value).toFixed(2)+' N·m';reset();};
  $('force').oninput=()=>{$('force-value').textContent=$('force').value+' N · 0.2 s';};
  function surface(canvas){const width=canvas.getBoundingClientRect().width,height=canvas.getBoundingClientRect().height,dpr=window.devicePixelRatio||1;
    if(canvas.width!==Math.round(width*dpr)||canvas.height!==Math.round(height*dpr)){canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);}const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,width,height);return {ctx,width,height};}
  function drawRobot(robot){const {ctx:c,width:w,height:h}=surface($(robot.name+'-scene')),scale=Math.min(430,w*.9),origin=w*.5,ground=h-48;
    const [x,v,theta]=robot.state,radius=robot.p.radius*scale,axle=ground-radius,body=robot.p.length*2*scale,col=colors[robot.name];
    c.font='11px system-ui';c.lineWidth=1;c.strokeStyle='#354353';c.fillStyle='#a5b3c4';c.beginPath();c.moveTo(0,ground);c.lineTo(w,ground);c.stroke();
    const left=x-w/(2*scale),right=x+w/(2*scale);for(let mark=Math.ceil(left/.25)*.25;mark<=right;mark+=.25){const px=origin+(mark-x)*scale;c.beginPath();c.moveTo(px,ground);c.lineTo(px,ground+7);c.stroke();c.textAlign='center';c.fillText(mark.toFixed(2)+' m',px,ground+23);}
    const home=origin-x*scale;if(home>15&&home<w-15){c.strokeStyle='#617183';c.setLineDash([3,4]);c.beginPath();c.moveTo(home,ground-18);c.lineTo(home,ground+3);c.stroke();c.setLineDash([]);}
    c.save();c.translate(origin,axle);c.rotate(theta);c.fillStyle=col;c.globalAlpha=.17;c.fillRect(-23,-body,46,body-8);c.globalAlpha=1;c.strokeStyle=col;c.lineWidth=2;c.strokeRect(-23,-body,46,body-8);c.fillStyle=col;c.fillRect(-14,-body+12,28,5);c.beginPath();c.arc(0,-body/2,4,0,Math.PI*2);c.fill();c.restore();
    c.save();c.translate(origin,axle);c.rotate(x/robot.p.radius);c.fillStyle='#10151c';c.strokeStyle=col;c.lineWidth=3;c.beginPath();c.arc(0,0,radius,0,Math.PI*2);c.fill();c.stroke();c.lineWidth=1;for(let k=0;k<3;k++){c.rotate(Math.PI/3);c.beginPath();c.moveTo(-radius+4,0);c.lineTo(radius-4,0);c.stroke();}c.restore();
    if(running&&time<pushEnd&&!robot.fallen){const sign=Math.sign(pushForce),cx=origin+Math.sin(theta)*body/2,cy=axle-Math.cos(theta)*body/2,from=cx-sign*85,to=cx-sign*30;c.strokeStyle='#edf2f7';c.lineWidth=2;c.beginPath();c.moveTo(from,cy);c.lineTo(to,cy);c.moveTo(to-sign*8,cy-5);c.lineTo(to,cy);c.lineTo(to-sign*8,cy+5);c.stroke();c.fillStyle='#edf2f7';c.textAlign='center';c.fillText(Math.abs(pushForce)+' N',(from+to)/2,cy-12);}
    c.fillStyle='#a5b3c4';c.textAlign='left';c.fillText('Side view · camera follows axle',16,20);
  }
  function drawPlot(){const {ctx:c,width:w,height:h}=surface($('plot')),pad={left:44,right:14,top:20,bottom:30},pw=w-pad.left-pad.right,ph=h-pad.top-pad.bottom;
    const maxAngle=Math.max(10,Math.ceil(Math.max(...robots.map(r=>r.peak))/10)*10),x=t=>pad.left+t/duration*pw,y=a=>pad.top+(maxAngle-a)/(2*maxAngle)*ph;
    c.font='11px system-ui';c.lineWidth=1;c.textAlign='right';for(const value of [-maxAngle,0,maxAngle]){c.strokeStyle='#2b3745';c.beginPath();c.moveTo(pad.left,y(value));c.lineTo(w-pad.right,y(value));c.stroke();c.fillStyle='#a5b3c4';c.fillText(value+'°',pad.left-8,y(value)+4);}
    c.textAlign='center';for(let t=0;t<=duration;t+=w<450?10:5){c.fillStyle='#a5b3c4';c.fillText(t+' s',x(t),h-8);}
    for(const event of events){c.strokeStyle='#617183';c.setLineDash([2,4]);c.beginPath();c.moveTo(x(event.time),pad.top);c.lineTo(x(event.time),h-pad.bottom);c.stroke();}c.setLineDash([]);
    robots.forEach((robot,index)=>{c.strokeStyle=colors[robot.name];c.lineWidth=2;c.setLineDash(index?[5,4]:[]);c.beginPath();let started=false;for(const row of history){if(row.angles[index]===null)continue;if(!started){c.moveTo(x(row.time),y(row.angles[index]));started=true;}else c.lineTo(x(row.time),y(row.angles[index]));}c.stroke();});c.setLineDash([]);
  }
  function render(){robots.forEach(drawRobot);drawPlot();
    if(time-lastUi<.1&&running)return;lastUi=time;
    robots.forEach(r=>{const n=r.name;$(n+'-angle').textContent=(r.state[2]*180/Math.PI).toFixed(2)+'°';$(n+'-position').textContent=r.state[0].toFixed(3)+' m';$(n+'-torque').textContent=(r.fallen?0:r.torque).toFixed(3)+' N·m';
      $(n+'-status').textContent=r.fallen?'Fallen · stopped':time===0?'Ready':Math.abs(r.state[2])<Math.PI/180&&Math.abs(r.state[1])<.03?'Balanced':'Recovering';$(n+'-status').className='status'+(r.fallen?' fallen':'');
      $(n+'-peak').textContent=r.peak.toFixed(2)+'°';$(n+'-travel').textContent=r.maxX.toFixed(3)+' m';$(n+'-effort').textContent=r.effort.toFixed(3);$(n+'-saturation').textContent=(r.saturatedSteps*step).toFixed(2)+' s';});$('clock').textContent=time.toFixed(2)+' / '+duration.toFixed(2)+' s';}
  function animate(stamp){if(last===null)last=stamp;const elapsed=Math.min(.1,(stamp-last)/1000);last=stamp;
    if(running){accumulator+=elapsed;while(accumulator>=step&&time<duration-1e-9){
      if(scheduled&&!scheduledDone&&time>=2-1e-10){scheduledDone=true;pushForce=scheduledDirection*Number($('force').value);pushEnd=2.2;if(pushForce!==0)events.push({time,force:pushForce});announce(`Scheduled ${pushForce} N push applied.`);}
      const force=time<pushEnd-1e-10?pushForce:0;robots.forEach(r=>r.step(force,step));time+=step;accumulator-=step;
      if(Math.round(time/step)%4===0)history.push({time,angles:robots.map(r=>r.fallen?null:r.state[2]*180/Math.PI)});
      if(robots.every(r=>r.fallen))break;}
      if(time>=duration-1e-9||robots.every(r=>r.fallen)){running=false;$('run').textContent='Run complete';$('run').disabled=true;announce(robots.every(r=>r.fallen)?'Both robots fell. Reset to try again.':`${duration}-second run complete. Reset to compare another run.`);}render();}
    requestAnimationFrame(animate);}
  document.addEventListener('visibilitychange',()=>{last=null;accumulator=0;if(document.hidden&&running){running=false;$('run').textContent='Resume';announce('Paused while this tab is hidden.');render();}});
  window.addEventListener('resize',render);reset();requestAnimationFrame(animate);
})();
