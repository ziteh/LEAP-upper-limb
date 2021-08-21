function [s1,s2] = invKin2R(x,y,l1,l2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

s=x^2+y^2;

theta12 = 2*atan(sqrt(((l1+l2)^2-s)/(s-(l1-l2)^2)));
theta11 = atan(y/x)+atan(l2 * sin(-theta12)/(l1+l2 * cos(-theta12)));
s1=[theta11,theta12];

theta22 = -2*atan(sqrt(((l1+l2)^2-s)/(s-(l1-l2)^2)));
theta21 = atan(y/x)+atan(l2 * sin(-theta22)/(l1+l2 * cos(-theta22)));
s2=[theta21,theta22];

end

