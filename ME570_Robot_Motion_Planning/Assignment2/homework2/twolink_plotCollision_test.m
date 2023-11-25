%function twolink_plotCollision_test()
%This function generates $30$ random configurations, loads the  points variable
%from the file  twolink_testData.mat (provided with the homework), and then
%display the results using  twolink_drawCollision to draw the manipulator in red
%if it is in collision, and green otherwise.
function twolink_plotCollision_test()
load('twolink_testData.mat');
NConfigurations=30;
theta=2*pi*randn(2,NConfigurations);
% NConfigurations=1;
% theta=[0;pi/2];
for iConfiguration=1:NConfigurations
    twolink_plotCollision(theta(:,iConfiguration),obstaclePoints);
    hold on
end
hold off
