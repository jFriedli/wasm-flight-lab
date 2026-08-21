export interface SimState {time:number;position:[number,number,number];velocity:[number,number,number];attitude:[number,number,number,number];forces:{thrust:[number,number,number];lift:[number,number,number];drag:[number,number,number]};mass:number;cg:[number,number,number]}
export const clamp=(value:number,min:number,max:number)=>Math.min(max,Math.max(min,Number.isFinite(value)?value:min));
export function metrics(mass:number,maxThrust=32){return{weight:mass*9.80665,thrustToWeight:maxThrust/(mass*9.80665),hoverThrottle:mass*9.80665/maxThrust};}

