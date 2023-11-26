function [flagPoints]=polygon_isVisible(vertices,indexVertex,points)
%Determines if a set of test points are visible by a paticular vertex of a polygon
%   Inputs are a [2 x N] set of vertices that define a polygon of the form
%   [x1..xn; y1..yn], an integer index to indicate which vertex to test,
%   and a [2 x N] set of coordinates that are test points of the form [x; y].
%   The function combines the output of two helper functions, to determine 
%   if the test point is invisible to the vertex by either self-occlusion
%   or collision with an edge of the polygon. Output is a [1 x N] boolean 
%   array where true means the test point is visible.
   
%   Error handling for indexVertex
    flagColumnRange=indexVertex<=size(vertices,2) && indexVertex>=1;
    if ~flagColumnRange
        error('indexVertex must be a valid column index of "vertices" between 1 and size(vertices,2)')
    end
    
    %Converting inputs to inputs for polygon_isSelfOccluded() and handling
    %the roll-over cases for the indexVertex input
    vertex = vertices(:,indexVertex);
    if indexVertex==1
        vertexPrev=vertices(:,size(vertices,2));
    else
        vertexPrev=vertices(:,indexVertex-1);
    end
    if indexVertex==size(vertices,2)
        vertexNext=vertices(:,1);
    else
        vertexNext=vertices(:,indexVertex+1);
    end
    selfOccludedPoints = polygon_isSelfOccluded(vertex,vertexPrev,vertexNext,points);
    
    %Running edge_isCollision for each test point against every edge of the
    %polygon. 'Break' is added to stop checking once the first collision is
    %found
    edgeCollisionPoints=false(1,size(points,2));
    for iPoints=1:size(points,2)
        %The first of the two lines to check collision for is always the
        %line from the test point to inputed vertex
        sightLine = [vertex points(:,iPoints)];
        for jEdges=1:size(vertices,2)
            if jEdges==size(vertices,2)
                edge = [vertices(:,jEdges) vertices(:,1)];
            else
                edge = [vertices(:,jEdges) vertices(:,jEdges+1)];
            end
            if edge_isCollision(sightLine,edge)==true
                edgeCollisionPoints(iPoints)=true;
                break
            end
        end
    end
    %helper functions return 'is invisible'; invert results for 'visible'
    flagPoints=and(~selfOccludedPoints,~edgeCollisionPoints);
end

