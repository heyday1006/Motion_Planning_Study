%function twolink_plot(theta,color)
%This function should use twolink_kinematicMap from the previous question
%together with the function polygon_draw from Homework 1 to draw the manipulator.
function twolink_plot(theta,color,thetaDot)
%if the homework1 files are saved in another folder, add search path
if exist('homework1','dir')
    addpath('homework1')
end
flagPlotVelocities=exist('thetaDot','var') && ~isempty(thetaDot);

%get transformed vertices
[vertexEffectorTransf,vertices1Transf,vertices2Transf]=twolink_kinematicMap(theta);
%remember if we are in figure hold or not
flagHold=ishold();
polygon_plot(vertices1Transf,color);
hold on
polygon_plot(vertices2Transf,color);
hold on
plot(vertexEffectorTransf(1),vertexEffectorTransf(2),'ko')
if flagPlotVelocities
    plotAngularVelocity(vertices1Transf,thetaDot(1))
    plotAngularVelocity(vertices2Transf,thetaDot(2))
    endEffectorDot=twolink_jacobian(theta,thetaDot);
    quiver(vertexEffectorTransf(1),vertexEffectorTransf(2),...
        endEffectorDot(1),endEffectorDot(2),...
        'AutoScale','off')
end
%restore figure hold (off) if necessary
if ~flagHold
    hold off
end

function plotAngularVelocity(verticesTransf,thetaDot)
nbVerticesHalfOffset=size(verticesTransf,2)/2-1;
gravCenter=mean(verticesTransf,2);
velCenter=verticesTransf*[...
    zeros(nbVerticesHalfOffset,1);-1;...
    1;zeros(nbVerticesHalfOffset,1)...
    ]*thetaDot;
velCenter=velCenter/norm(velCenter);
quiver(gravCenter(1),gravCenter(2),velCenter(1),velCenter(2),'k',...
    'AutoScale','off')
