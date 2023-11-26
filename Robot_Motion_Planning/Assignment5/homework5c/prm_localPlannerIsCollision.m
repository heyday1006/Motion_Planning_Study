%function [flag]=prm_localPlannerIsCollision(world,x1,x2,maxDistEdgeCheck)
%Generates @boxIvory2 NPoints equispaced points on the line between x1
%and x2, such that the maximum distance between two consecutive
%samples is less than maxDistEdgeCheck. Note that the value of
%NPoints is determined by the distance between the two points and
%maxDistEdgeCheck.
%**This function returns an ``all-or-nothing'' result, meaning that the edge
%between x1 and x2 even if only one of the samples is in collision
function [flag]=prm_localPlannerIsCollision(world,x1,x2,maxDistEdgeCheck)
xSamples=line_linspace(x2-x1,x1,0,1,ceil(1e-6+norm(x2-x1)/maxDistEdgeCheck));
flagIsCollisions=sphereworld_isCollision(world,xSamples);
flag=~all(flagIsCollisions(:)==0);
