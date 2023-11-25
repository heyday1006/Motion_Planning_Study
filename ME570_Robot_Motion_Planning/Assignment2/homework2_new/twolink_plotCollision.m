%function twolink_plotCollision(theta,points)
%This function should: enumerate  the points specified by  @x   points as black
%asterisks.  twolink_isCollision for determining if each configuration is a
%collision or not.  twolink_plot to plot the manipulator for all configurations,
%using a red color when the manipulator is in collision, and green otherwise.
%enumerate
%INPUT:     theta: [2xNTheta] each column represents a configuration of two-link manipulator
%           points:[2xNPoints]array of obstacles coordinates
function twolink_plotCollision(theta,points)
colorMap = containers.Map([true false],{'red', 'green'});
[flagTheta]=twolink_isCollision(theta,points);
for iTheta = 1:size(theta,2)
    twolink_plot(theta(:,iTheta),colorMap(flagTheta(iTheta)));
    hold on
end
plot(points(1,:),points(2,:),'k*');
axis equal
hold off