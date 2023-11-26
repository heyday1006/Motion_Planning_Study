%function [UTheta]=twolink_potential_total(thetaEval,world,potential)
%Compute the potential $U$ pulled back through the kinematic map of the two-link
%manipulator, i.e., $U(  Wp_ eff(vec17E ))$, where $U$ is defined as in Question 
%q:total-potential, and $  Wp_ eff( )$ is the position of the end effector in the
%world frame as a function of the joint angles $vec17E = _1\\ _2$.
%INPUTS:    thetaEval [2x1]
%           potential [1x1] struct
%OUTPUTS:   uEvalTheta: potential gradient evaluated at xEval
function [UTheta]=twolink_potential_total(thetaEval,world,potential)
xEval = (rot2d(sum(thetaEval))+rot2d(thetaEval(1)))*[5;0];
UTheta = potential_total(xEval,world,potential);