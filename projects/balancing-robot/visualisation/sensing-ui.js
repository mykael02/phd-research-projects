/* Numeric-only SVG traces; rendering never changes sensor state or noise samples. */
(function(root){'use strict';
const $=id=>document.getElementById(id),units=['m','m/s','deg','deg/s'];let lastTick=-1,lastVariable=-1,lastSession=null;
function options(){return {mode:$('observer').value,angleNoiseDeg:Number($('angle-noise').value),gyroNoiseDegS:Number($('gyro-noise').value),delayMs:Number($('delay').value),seed:Number($('seed').value)};}
function labels(){const ideal=$('observer').value==='ideal';for(const id of ['angle-noise','gyro-noise','delay','seed'])$(id).disabled=ideal;
 $('angle-noise-value').textContent=Number($('angle-noise').value).toFixed(2)+'\u00b0';$('gyro-noise-value').textContent=Number($('gyro-noise').value).toFixed(1)+'\u00b0/s';$('delay-value').textContent=$('delay').value+' ms';
 $('sensor-summary').textContent=ideal?'Perfect state is fed to the controllers; sensing noise and delay are bypassed.':'Gaussian angle/gyro surrogate, not a physical accelerometer model. One initial packet is held during startup; only newly arriving packets update the filter.';
}
function plot(rows,index,variable){const factor=variable>=2?180/Math.PI:1,source=rows.filter(row=>!row.robots[index].fallen);if(!source.length)return '';
 const step=Math.max(1,Math.ceil(source.length/700)),selected=source.filter((row,i)=>i%step===0||i===source.length-1),all=selected.flatMap(row=>['state','measurement','estimate'].map(key=>row.robots[index][key][variable]*factor));
 let low=Math.min(...all),high=Math.max(...all);const pad=Math.max((high-low)*.1,variable>=2?.15:.003);low-=pad;high+=pad;
 const w=600,h=250,left=60,right=18,top=18,bottom=35,tmax=Math.max(1,rows.at(-1).time);const x=t=>left+t/tmax*(w-left-right),y=v=>top+(high-v)/(high-low)*(h-top-bottom);
 let svg=`<svg viewBox="0 0 ${w} ${h}" xmlns="http://www.w3.org/2000/svg" aria-label="True versus measured and estimated ${units[variable]}"><rect width="600" height="250" fill="white"/>`;
 for(let i=0;i<5;i++){const value=low+(high-low)*i/4,py=y(value);svg+=`<line x1="${left}" x2="${w-right}" y1="${py}" y2="${py}" stroke="#e1e6ea"/><text x="${left-8}" y="${py+4}" text-anchor="end" font-size="11" fill="#536474">${value.toFixed(variable>=2?1:3)}</text>`;}
 for(let i=0;i<5;i++){const time=tmax*i/4;svg+=`<text x="${x(time)}" y="${h-12}" text-anchor="middle" font-size="11" fill="#536474">${time.toFixed(1)} s</text>`;}
 for(const [key,color,width,dash] of [['measurement','#b0bac3',1,''],['state','#18364d',2,''],['estimate','#9450c3',2,'6 4']]){const points=selected.map(row=>`${x(row.time).toFixed(2)},${y(row.robots[index][key][variable]*factor).toFixed(2)}`).join(' ');svg+=`<polyline points="${points}" fill="none" stroke="${color}" stroke-width="${width}" stroke-dasharray="${dash}"/>`;if(selected.length===1)svg+=`<circle cx="${x(selected[0].time)}" cy="${y(selected[0].robots[index][key][variable]*factor)}" r="3" fill="${color}"/>`;}
 return svg+'</svg>';
}
function render(session,force=false){const variable=Number($('trace-state').value);if(session===lastSession&&variable===lastVariable&&(session.ticks===lastTick||!force&&session.ticks-lastTick<10))return;lastTick=session.ticks;lastVariable=variable;lastSession=session;
 ['pid','lqr'].forEach((name,i)=>{$(name+'-trace').innerHTML=plot(session.history,i,variable);const r=session.robots[i],scale=variable>=2?180/Math.PI:1,error=(r.estimate[variable]-r.state[variable])*scale;$(name+'-error').textContent=`Estimate error now: ${error.toFixed(3)} ${units[variable]} | observation age: ${r.readout.ageMs.toFixed(0)} ms${r.fallen?' at the stopped state':''}`;});
}
root.SensingUI={options,labels,render,plot};
})(typeof globalThis!=='undefined'?globalThis:this);
