function [outputArg] = makeUpperLimb

L(1)=Link([0,0,0,pi/2]);
L(2)=Link([0,0.2,0.5,0]);
L(3)=Link([0,0,0.5,0]);

outputArg = SerialLink(L,'name','Upper-Limb');

end

