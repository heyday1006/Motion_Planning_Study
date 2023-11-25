%function [flagPoints]=polygon_isVisible(vertices,indexVertex,testPoints)
%Checks whether a point $p$ is visible from a vertex $v$ of a polygon. In order
%to be visible, two conditions need to be satisfied: enumerate  point $p$ should
%not be self-occluded with respect to the vertex $v$\\ (see
%polygon_isSelfOccluded).  segment $p$--$v$ should not collide with  any of the
%edges of the polygon (see edge_isCollision). enumerate
%	INPUT: vertices:     [2xnbVertices] defines a polygon
%          indexVertex:  [1] index of a specific vertex
%          testPoints:   [2xnbPoints] a set of points to be tested
%   OUTPUT:flagPoints:   [1xnbPoints] each is logical true if the points is
%                        visible from the indexVertex
function [flagPoints]=polygon_isVisible(vertices,indexVertex,testPoints)
%Note that, with the definitions of edge collision and self-occlusion given in
%the previous questions, a vertex should be visible from the previous and
%following vertices in the polygon.
nbPoints = size(testPoints,2);
nbVertices = size(vertices,2);
flagPoints = false(1,nbPoints); 
%circular indexing as 1->2->3->1
circIndex = @(x, n) (1 + mod(x-1, n));
for iPoint = 1:nbPoints 
    vertex = vertices(:,indexVertex);
    %visibility condition 1: the vertex-point is not self-occluded 
    vertexPrev = vertices(:,circIndex(indexVertex-1,nbVertices));
    vertexNext = vertices(:,circIndex(indexVertex+1,nbVertices));
    isSelfOccluded = polygon_isSelfOccluded(vertex,vertexPrev,vertexNext,testPoints(:,iPoint));
    %visibility condition 2: the vertex-point is not intersecting with any
    %ppolygon edges
    isCollision = false;
    for iEdge = 1:nbVertices
        edgeStart = iEdge;
        edgeEnd = circIndex(iEdge+1,nbVertices);
        tmp = edge_isCollision(vertices(:,[edgeStart,edgeEnd]),[vertices(:,indexVertex) testPoints(:,iPoint)]);
        isCollision = (isCollision || tmp);
    end
    flagPoints(iPoint) = (~isSelfOccluded && ~isCollision);
end