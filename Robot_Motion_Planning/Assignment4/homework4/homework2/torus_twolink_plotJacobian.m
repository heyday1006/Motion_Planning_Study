%function torus_twolink_plotJacobian()
%For each one of the curves used in Question~ q:torusDrawChartsCurves, do the
%following: itemize  line_linspace to compute the array @boxIvory2 thetaPoints
%for the curve;  each one of the configurations given by the columns of
%@boxIvory2 thetaPoints: enumerate  twolink_plot to plot the two-link
%manipulator.  twolink_jacobian to compute the velocity of the end effector, and
%then use quiver to draw that velocity as an arrow starting from the end
%effector's position. enumerate itemize The function should produce a total of
%four windows (or, alternatively, a single window with four subplots), each
%window (or subplot) showing all the configurations of the manipulator
%superimposed on each other. You can insert a @boxIvory2 pause command in the
%loop for drawing the manipulator, in order to obtain a ``movie-like''
%presentation of the motion.
function torus_twolink_plotJacobian()

% ~For each window (or subplot), use the color of the corresponding curve as used
%in Question~ q:torusDrawChartsCurves.

curve1 = [3/4*pi; 0];
curve2 = [3/4*pi; 3/4*pi];
curve3 = [-3/4*pi; 3/4*pi];
curve4 = [0; -3/4*pi];
a = {curve1, curve2, curve3, curve4};
b = [-1; -1];
colorList = {'b', 'k', 'm', 'r'};
NPoints = 31;
figure(1)
for iCurve = 1:size(a,2)
    subplot(2,2,iCurve)
    subPlot(a{iCurve},b,NPoints,colorList{iCurve})
    title(['Curve' num2str(iCurve)])
    hold off
end

function subPlot(a,b,NPoints,color)
%generate point along theta(t) in t = [0,1]
[theta] = line_linspace(a,b,0,1,NPoints);
thetaDot = repmat(a,1,NPoints);
%plot the manipulator for each configuration 
for iTheta = 1:NPoints
    twolink_plot(theta(:,iTheta),color,thetaDot)
    axis equal
    hold on
    pause(.3)
end