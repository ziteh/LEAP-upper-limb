function [outputArg] = makeUpperLimb

L(1) = Link([0,0,1,0]);
L(2) = Link([0,0,1,0]);

outputArg = SerialLink(L,'name','Upper-Limb');

end

