(function(root){'use strict';
const bank=typeof module!=='undefined'&&module.exports?require('./payload-gains.js'):PAYLOAD_GAINS;
function gain(m,h){if(!Number.isFinite(m)||!Number.isFinite(h)||m<0||m>1.5||h<.1-1e-9||h>.4+1e-9)throw new Error('Estimated payload is outside the supported range');const k=m.toFixed(4)+'/'+(m===0?.18:h).toFixed(4);if(!bank[k])throw new Error('No exact gain design for this payload setting');return bank[k].slice();}
function estimate(variant,load){let m=load.mass_kg,h=load.height_m;if(variant==='fixed'){m=0;h=.18;}else if(variant==='mass_low')m*=.8;else if(variant==='mass_high')m*=1.2;else if(variant==='height_low')h-=.04;else if(variant==='height_high')h+=.04;else if(variant!=='exact')throw new Error('Unknown LQR variant');return {mass_kg:Number(m.toFixed(8)),height_m:Number(h.toFixed(8))};}
const api={gain,estimate};root.PayloadLQR=api;if(typeof module!=='undefined'&&module.exports)module.exports=api;
})(typeof globalThis!=='undefined'?globalThis:this);
