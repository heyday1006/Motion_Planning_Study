%plot the time derivative of end effector motion
function twolink_jacobian_plot()
a = {[1;0],[0;1]};
b = 0;
theta = [[0;0],[0;pi/2],[pi;pi/2],[pi;pi]];
theta = repmat(theta,1,2);
thetaDot = horzcat(repmat(a{1},1,4),repmat(a{2},1,4));
%8 configuration, first four with a{1}, second four with a{2}
[vertexEffectorDot]=twolink_jacobian(theta,thetaDot);
color=['c','r','g','b','c','r','g','b'];
figure(1)
for iTheta = 1:size(theta,2)
    subplot(2,4,iTheta)
    twolink_plot(theta(:,iTheta),color(iTheta),thetaDot(:,iTheta))
    title(['case ',num2str(iTheta)])
    axis equal
    %hold on
end


