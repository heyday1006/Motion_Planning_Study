%function [flagTheta]=twolink_isCollision(theta,points)
%For each specified configuration, returns  true if  any of the links of the
%manipulator collides with  any of the points, and  false otherwise. Use the
%function polygon_isCollision to check if each link of the manipulator is in
%collision.
%INPUT:     theta: [2xNTheta] each column represents a configuration of two-link manipulator
%           points:[2xNPoints]array of obstacles coordinates
%output:    flagTheta: [1xNTheta] true if the corresponding configuration
%                                 causes collisions
function [flagTheta]=twolink_isCollision(theta,points)
%For this question,  do not consider self-collision (i.e., if the two polygons
%overlap but they do not cover any of the points, then it is not a collision).

%if the homework1 files are saved in another folder, add search path
if exist('homework1','dir')
    addpath('homework1')
end
flagTheta=false(1,size(theta,2));
%vertices of two polygons representing two-link manipulator links after the
%transformation
for iTheta = 1:size(theta,2)
    [~,vertices1Transf,vertices2Transf]=twolink_kinematicMap(theta(:,iTheta));
    [flagPoints_v1]=polygon_isCollision(vertices1Transf,points);
    flagLink_v1 = all(flagPoints_v1==0); %false-> collision
    [flagPoints_v2]=polygon_isCollision(vertices2Transf,points);
    flagLink_v2 = all(flagPoints_v2==0); %false-> collision
    flagTheta(iTheta) = ~(flagLink_v1&&flagLink_v2); %true there is a collision
end