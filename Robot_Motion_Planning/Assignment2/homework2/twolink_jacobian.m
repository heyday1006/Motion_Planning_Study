%function [vertexEffectorDot]=twolink_jacobian(theta,thetaDot)
%Implement the map for the Jacobian of the position of the end effector with
%respect to the joint angles as derived in Question  q:jacobian-effector.
%INPUT:     theta: [2xNTheta] each column contains joint angles for 2-link
%                             manipulator
%           thetaDot: [2xNTheta] each column contains the time derivative
%                                of theta
%OUTPUT:    vertexEffectorDot: [2xNTheta] each column contains the time 
%                                         derivative of end effector position
function [vertexEffectorDot]=twolink_jacobian(theta,thetaDot)
NTheta = size(theta,2);
vertexEffectorDot = zeros(2,NTheta);
for iTheta = 1:NTheta
    jacobian = [-5*sin(sum(theta(:,iTheta)))...
        -5*sin(sum(theta(:,iTheta)))-5*sin(theta(2,iTheta));...
        5*cos(sum(theta(:,iTheta)))...
        5*cos(sum(theta(:,iTheta)))+5*cos(theta(2,iTheta))];
    vertexEffectorDot(:,iTheta) = jacobian * thetaDot(:,iTheta);
end
