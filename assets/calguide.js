// Model-zero (0°) reference for each joint, in hardware order (IDs 10..14, 20..24, 30..34).
// Images are renders of the XGO-Duck CAD assembly, which is posed at Microduck's model zero
// (every joint 0°). In each render the part moved by that joint is teal and an orange bar,
// placed in the CAD, marks its 0° line. IDs 10-14 are the robot's left leg.
(function(){
const leg=(side,id)=>{
  const s=side.toLowerCase();
  return [
    {name:`${side} hip yaw`,view:'Bottom view, front to the right',
     text:`The ${s} leg is not twisted: the foot points straight forward, parallel to the orange line.`},
    {name:`${side} hip roll`,view:'Front view (facing the duck)',
     text:`The ${s} leg hangs straight down along the orange line: not splayed outward or tucked under the body.`},
    {name:`${side} hip pitch`,view:`${side} side view`,
     text:'The thigh is vertical: the leg below it lines up with the orange line, not swung forward or back.'},
    {name:`${side} knee`,view:`${side} side view`,
     text:'The knee is straight: the shank is vertical, in line with the thigh, along the orange line.'},
    {name:`${side} ankle`,view:`${side} side view`,
     text:'The foot is square to the shank: the sole sits flat along the horizontal orange line.'},
  ].map((j,k)=>({...j,img:`cal/j${id+k}.png`}));
};
const JOINTS=[
  ...leg('Left',10),
  ...leg('Right',20),
  {name:'Neck pitch',view:'Side view, front to the right',img:'cal/j30.png',
   text:'The neck is vertical: both neck pivots sit on the orange line.'},
  {name:'Head pitch',view:'Side view, front to the right',img:'cal/j31.png',
   text:'With the neck vertical, the head is level: its underside runs along the horizontal orange line.'},
  {name:'Head yaw',view:'Top view, front to the right',img:'cal/j32.png',
   text:'The head faces straight forward: its centre line follows the orange line.'},
  {name:'Head roll',view:'Front view (facing the duck)',img:'cal/j33.png',
   text:'The head is level side to side: its lower edge runs along the horizontal orange line.'},
  {name:'Mouth',view:'Side view, front to the right',img:'cal/j34.png',
   text:'The beak is fully closed: upper and lower beak meet along the orange line.'},
];
window.CalGuide={JOINTS};
})();
