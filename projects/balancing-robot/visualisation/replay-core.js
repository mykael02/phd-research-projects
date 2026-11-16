/* Pure helpers shared by the replay and its data-contract checks. */
(function(root){'use strict';
  function sample(rows,time){let low=0,high=rows.length-1;while(low<high){const mid=Math.ceil((low+high)/2);if(rows[mid].time<=time+1e-9)low=mid;else high=mid-1;}return rows[low];}
  function bodyCentre(state,radius,length){return [state[0]+length*Math.sin(state[2]),radius+length*Math.cos(state[2])];}
  function forceAt(time,force,end,fallen){return time>=2&&time<2.2&&(!fallen||time<end)?force:0;}
  const api={sample,bodyCentre,forceAt};root.ReplayCore=api;if(typeof module!=='undefined')module.exports=api;
})(typeof globalThis!=='undefined'?globalThis:this);
