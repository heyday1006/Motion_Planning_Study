%function [gradUTheta]=twolink_potential_totalGrad(thetaEval,world,potential)
%Compute the gradient of the potential $U$ pulled back through the kinematic map
%of the two-link manipulator, i.e., $ _vec17E  U(  Wp_ eff(vec17E ))$.
%INPUTS:    thetaEval [2x1]
%           potential [1x1] struct
%OUTPUTS:   gradUEvalTheta: potential value evaluated at xEval
function [gradUTheta]=twolink_potential_totalGrad(thetaEval,world,potential)
xEval = (rot2d(sum(thetaEval))+rot2d(thetaEval(1)))*[5;0];
gradUTheta = twolink_jacobianMatrix(thetaEval)'...
    *potential_totalGrad(xEval,world,potential);