%function [Jtheta]=twolink_jacobianMatrix(theta)
%Compute the matrix representation of the Jacobian of the position of the end
%effector with respect to the joint angles as derived in Question 
%q:jacobian-matrix.
%INPUTS:    theta [2x1]: joint angles
%OUTPUTS:   Jtheta [2x2]: Jacobian 
function [Jtheta]=twolink_jacobianMatrix(theta)
Jtheta = [-5*sin(sum(theta(:)))-5*sin(theta(1))...
        -5*sin(sum(theta(:)));...
        5*cos(sum(theta(:)))+5*cos(theta(1))...
        5*cos(sum(theta(:)))];  
% % derivative w.r.t (theta1+theta2)
% der1 = [0 -1;1 0]*rot2d(sum(theta(:)))*[1 1]*[5;0];
% %derivative w.r.t (theta1)
% der2 = [0 -1;1 0]*rot2d(theta(1))*[1 0]*[5;0];
% Jtheta = der1+der2;

