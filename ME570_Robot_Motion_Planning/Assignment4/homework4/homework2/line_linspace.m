%function [thetaPoints]=line_linspace(a,b,tMin,tMax,NPoints)
%Generates a discrete number of  NPoints points along the curve $vec17E (t)=(
%a(1)  t+ b(1), a(2) t+ b(2))  R^2$ for $t$ ranging from  tMin to  tMax.
function [thetaPoints]=line_linspace(a,b,tMin,tMax,NPoints)
t=linspace(tMin,tMax,NPoints);
thetaPoints=a*t+b*ones(1,NPoints);
