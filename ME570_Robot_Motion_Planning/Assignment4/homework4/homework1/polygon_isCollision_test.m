%function polygon_isCollision_test()
%This function is the same as polygon_isVisible_test, but instead of step 
%item:test-visibility, use the following: enumerate  whether each point in 
%testPointsWithPolygon is in collision with the polygon or not using
%polygon_isCollision.  each point in  testPointsWithPolygon in green if it is not
%in collision, and red otherwise. enumerate Moreover, increase the number of test
%points from $5$ to $100$ (i.e.,  testPoints should have dimension [ 2 $ $ 
%100]).
function polygon_isCollision_test()
testPoints=zeros(2,100);%variable preallocation to store the coordinates of test points.
testPoints(1,:)=5*rand(1,100);%assignment for x coordinate by generating uniformly distributed random number in [0,5].
testPoints(2,:)=4*rand(1,100)-2;%assignment for y coordinate by generating uniformly distributed random number in [-2,2].

[vertices1,vertices2]=twolink_polygons();%call the function twolink_polygons to get the vertices information of two polygons.
verticesTriangle=zeros(2,3);%memory pre-allocation to store three coordinates of triangle vertices.
verticesTriangle(1,:)=5*rand(1,3);%generate the x coordinates at random.
verticesTriangle(2,:)=4*rand(1,3)-2;%generate the y coordinates at random.

%call the function CollisionStatePlot to draw the points as homework
%assignment requires
CollisionStatePlot(vertices1,testPoints);
CollisionStatePlot(fliplr(vertices1),testPoints);
CollisionStatePlot(vertices2,testPoints);
CollisionStatePlot(fliplr(vertices2),testPoints);
CollisionStatePlot(verticesTriangle,testPoints);
CollisionStatePlot(fliplr(verticesTriangle),testPoints);

function CollisionStatePlot(vertices,testPoints)
addpath(genpath('../homework1'))
figure

testPointsWithPolygon=[vertices,testPoints];%concatenate testPoints with polygon vertices.
flagPoints=polygon_isCollision(vertices,testPointsWithPolygon);%get the collision state of given test points and polygon.
flagNum=size(flagPoints,2);%get the number of flagPoints

polygon_plot(vertices,'b');%plot the given polygon
hold on;

%this loop aims at traversing the elements in flagPoints to draw the points
%depending on their collision states.
for flagInd=1:flagNum
    
    if flagPoints(flagInd)%the condition that the point is in collision (plotted in red star).
        
        plot(testPointsWithPolygon(1,flagInd),testPointsWithPolygon(2,flagInd),'r*');
        
    else%the condition that the point is not in collision (plotted in green circle).
        
        plot(testPointsWithPolygon(1,flagInd),testPointsWithPolygon(2,flagInd),'go');
        
    end
    
    hold on;
end

set(gca,'xtick',[]);
set(gca,'ytick',[]);
hold off;

