%function [vertexEffectorDot]=twolink_jacobian(theta,thetaDot)
%Implement the map for the Jacobian of the position of the end effector with
%respect to the joint angles as derived in Question  q:jacobian-effector.
%INPUT:     theta: [2xNTheta] each column contains joint angles for 2-link
%                             manipulator
%           thetaDot: [2xNTheta] each column contains the time derivative
%                                of theta
%OUTPUT:    vertexEffectorDot: [2xNTheta] each column contains the time 
%            
function [vertexEffectorDot]=twolink_jacobian(theta,thetaDot)
NTheta = size(theta,2);
vertexEffectorDot = zeros(2,NTheta);
for iTheta = 1:NTheta
    jacobian = [-5*sin(sum(theta(:,iTheta)))-5*sin(theta(1,iTheta))...
        -5*sin(sum(theta(:,iTheta)));...
        5*cos(sum(theta(:,iTheta)))+5*cos(theta(1,iTheta))...
        5*cos(sum(theta(:,iTheta)))];   
    vertexEffectorDot(:,iTheta) = jacobian * thetaDot(:,iTheta);
    %derivative w.r.t (theta1+theta2)
%     der1 = [0 -1;1 0]*rot2d(sum(theta(:,iTheta)))*([1 1]*thetaDot(:,iTheta))*[5;0];
%     %derivative w.r.t (theta1)
%     der2 = [0 -1;1 0]*rot2d(theta(1,iTheta))*([1 0]*thetaDot(:,iTheta))*[5;0];
%     vertexEffectorDot(:,iTheta)=der1+der2;
end