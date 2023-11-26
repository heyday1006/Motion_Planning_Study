%function [flagPoints]=polygon_isCollision(vertices,testPoints)
%Checks whether the a point is in collsion with a polygon (that is, inside for a
%filled in polygon, and outside for a hollow polygon).
%	INPUT: vertices:     [2xnbVertices] defines a polygon
%          testPoints:   [2xnbPoints] a set of points to be tested
%   OUTPUT:flagPoints:   [1xnbPoints] each is logical true if the points is
%                        in collision with the polygon
function [flagPoints]=polygon_isCollision(vertices,testPoints)
nbPoints = size(testPoints,2);
nbVertices = size(vertices,2);
flagPoints = false(1,nbPoints);
%the point is in collision with polygon if it is not visible from any
%polygon vertex
for iVertex = 1:nbVertices
    flagPoints = flagPoints + polygon_isVisible(vertices,iVertex,testPoints);
end
flagPoints = (flagPoints==0);

