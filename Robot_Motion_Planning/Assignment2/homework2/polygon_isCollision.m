function [flagPoints]=polygon_isCollision(vertices,points)
%Determines if a set of points lies inside or outside of a given polygon
%   Inputs are a [2 X N] set of vertices of the form [x1..xn; y1...yn] and
%   a [2 x N] set of test points of the same form. Uses lower level
%   function polygon_isVisible to determine if test point is visible to any
%   vertex of the polygon, and thus inside or outside. Note: for a hollow
%   polygon, 'outside' or non-collision is inside the polygon. Output is
%   a [1 X N] boolean array where true means the point is colliding. 
    flagPoints=true(1,size(points,2));
    for iVertices=1:size(vertices,2)
        flagPointVertex=polygon_isVisible(vertices,iVertices,points);
        flagPoints=and(flagPoints,~flagPointVertex);
    end
end

