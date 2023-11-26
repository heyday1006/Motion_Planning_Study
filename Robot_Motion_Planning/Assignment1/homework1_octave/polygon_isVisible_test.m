%function polygon_isVisible_test()
%This function should perform the following operations: enumerate  an array 
%testPoints with dimensions [ 2 $ $  5] containing points generated uniformly at
%random using rand and scaled to approximately occupy the rectangle $[0,5]
%[-2,2]$ (i.e., the $x$ coordinates of the points should fall between $0$ and
%$5$, while the $y$ coordinates between $-2$ and $2$).  the coordinates 
%vertices1 and  vertices2 of two polygons from twolink_polygons.  two  logical
%vectors  flag1 and  flag2 by calling polygon_isVisible using  testPoints and,
%respectively,  vertices1 and  vertices2 as argument. item:test-polygon For each
%polygon  vertices1,  vertices2, display a separate figure using the following:
%enumerate  the array  testPointsWithPolygon by concatenating  testPoints with
%the coordinates of the polygon (i.e., the coordinates of the polygon become also
%test points).  the polygon (use polygon_plot). item:test-visibility For each
%vertex $v$ in the polygon: enumerate  the visibility of each point in 
%testPointsWithPolygon with respect to that polygon (using polygon_isVisible). 
%lines from the vertex $v$ to each point in  testPointsPolygon according in green
%if the corresponding point is visible, and in red otherwise. enumerate enumerate
% the order of the vertices in  vertices1,  vertices2 using the function fliplr. 
%item item:test-polygon above with the reversed edges. enumerate
function polygon_isVisible_test()
%The function should display four separate figures in total, each one with a
%single polygon and lines from each vertex in the polygon, to each point.
%addpath(genpath('../homework1_autograders_solutions'))
%generate 5 random test points with a scale in [0,5]x[-2,2]
lowerBound = [0;-2];
upperBound = [5;2];
testPoints = (upperBound-lowerBound).*rand(2,5) + lowerBound;
[vertices1,vertices2] = twolink_polygons();
%reverse the order of vertices1 and vertices2
vertices1Reverse = fliplr(vertices1);
vertices2Reverse = fliplr(vertices2);
polygonList = {vertices1,vertices2,vertices1Reverse,vertices2Reverse};
text1 = {'t1','t2','t3','t4','t5','v1','v2','v3','v4'};
text3 = {'t1','t2','t3','t4','t5','v4','v3','v2','v1'};
text2 = {'t1','t2','t3','t4','t5','v1','v2','v3','v4','v5','v6','v7','v8','v9','v10','v11','v12'};
text4 = {'t1','t2','t3','t4','t5','v12','v11','v10','v9','v8','v7','v6','v5','v4','v3','v2','v1'};
textList = {text1,text2,text3,text4};
titleList = {'vertices1','vertices2','vertices1Reverse','vertices2Reverse'};
for iPolygon = 1:size(polygonList,2)
    vertices = polygonList{iPolygon};
    %create test points by concatenating testPoints and polygon vertices
    testPointsWithPolygon = cat(2,testPoints,vertices);
    figure(iPolygon)
    polygon_plot(vertices,'b')
    hold on
    %plot and label each test point
    plot(testPointsWithPolygon(1,:),testPointsWithPolygon(2,:),'ro')
    text(testPointsWithPolygon(1,:),testPointsWithPolygon(2,:),textList{1,iPolygon},'VerticalAlignment','bottom','HorizontalAlignment','right')
    hold on
    nbVertices = size(vertices,2);
    nbTestPointsWithPolygon = size(testPointsWithPolygon,2);
    %for each polygon vertex, test the visibility of each points w.r.t the
    %polygon
    for indexVertex = 1:nbVertices
        flagPoints=polygon_isVisible(vertices,indexVertex,testPointsWithPolygon);
        pointStart = repmat(vertices(:,indexVertex),1,nbTestPointsWithPolygon);
        pointEnd = testPointsWithPolygon;
        %plot vertex-point line in green if it is visible, and red if not
        plotLinesFlag(pointStart,pointEnd,flagPoints);
    end
    title(titleList{iPolygon})
    hold off
end
