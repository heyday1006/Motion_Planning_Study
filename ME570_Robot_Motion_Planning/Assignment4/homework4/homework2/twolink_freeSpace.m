%create grid xx=[0:2:360],yy=[0:2:360],each grid point is a
%configuration of the corresponding coordinate value, the value of the grid
%represents if it collides with obstacle points
function twolink_freeSpace()
load('twolink_testData.mat'); %load obstacle points
[thetaPoints]=line_linspace(1,0,0,2*pi,180);
freeSpaceMap = false(180,180);
for iTheta = 1:size(thetaPoints,2)
    for jTheta = 1:size(thetaPoints,2)
        theta = [thetaPoints(iTheta);thetaPoints(jTheta)];
        freeSpaceMap(iTheta,jTheta) = twolink_isCollision(theta,obstaclePoints);
%         [iTheta,jTheta]
    end   
end
imagesc(imresize(freeSpaceMap,2)')
set(gca,'YDir','normal') 
grid on
grid minor
set(gca,'xtick',[0:20:360])
set(gca,'ytick',[0:20:360])  
colorbar
xlabel('\theta_1')
ylabel('\theta_2')

% imshow(imread("2_optional_map.jpg"))
% text('units','pixels','position',[250 20],'fontsize',15,'string','\theta_2')
% text('units','pixels','position',[20 240],'fontsize',15,'string','\theta_1')