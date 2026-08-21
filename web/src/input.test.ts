import{describe,expect,it}from'vitest';import{DEFAULT_CALIBRATION,gamepadCommands,loadCalibration,normalizeAxis,shape,slew}from'./input';
describe('input pipeline',()=>{
 it('expo preserves endpoints and is monotonic',()=>{expect(shape(-1,.05,.5)).toBe(-1);expect(shape(0,.05,.5)).toBe(0);expect(shape(1,.05,.5)).toBe(1);const x=Array.from({length:201},(_,i)=>shape(i/100-1,.05,.5));expect(x.every((v,i)=>i===0||v>=x[i-1])).toBe(true)});
 it('slew depends on elapsed time',()=>{const once=slew(0,1,2,4,.04);let many=0;for(let i=0;i<10;i++)many=slew(many,1,2,4,.004);expect(many).toBeCloseTo(once,12)});
 it('normalizes asymmetric calibrated axes',()=>expect(normalizeAxis(.6,{source:0,min:-.8,center:.1,max:1,invert:false,deadzone:0,expo:0})).toBeCloseTo(.5556,3));
 it('fails safe on invalid gamepad values',()=>{const c=gamepadCommands({axes:[Number.NaN]},DEFAULT_CALIBRATION);expect(c.roll).toBe(0);expect(c.throttle).toBe(0)});
 it('rejects malformed persisted data',()=>{const c=loadCalibration({getItem:()=>'{broken'});expect(c.version).toBe(1);expect(c.axes.roll.source).toBe(0)});
});
