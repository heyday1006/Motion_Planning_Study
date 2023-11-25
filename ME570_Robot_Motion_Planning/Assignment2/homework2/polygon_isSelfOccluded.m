function [flagPoint] = polygon_isSelfOccluded(vertex,vertexPrev,vertexNext,point)
%Determines if a set of test points are Self Occluded, or non-intersecting but within a polygon 
%   Inputs are three [2 x 1] vectors that represent a vertex on a polygon,
%   the vertex that preceeded it, and the vertex that follows in
%   construction, and a [2 x N] matrix of test points of the form [x1..xn; y1...yn].
%   Function handles both counterclockwise or filled-in polygons and clockwise or
%   hollow polygons. Output is [1 x N] boolean array where true is a self
%   occluded test point.
nbPoints=size(point,2);
flagPoint=false(1,nbPoints);
    for iPoints = 1:nbPoints
        angleNext=atan2(vertexNext(2)-vertex(2),vertexNext(1)-vertex(1));
        anglePrev=atan2(vertexPrev(2)-vertex(2),vertexPrev(1)-vertex(1));
        anglePoint=atan2(point(2,iPoints)-vertex(2),point(1,iPoints)-vertex(1));
        %maintaining the vertices gives a consistent relationship of what
        %angles constitue self-occlusion for both counterclockwise and
        %clockwise objects, but is dependent on whether the previous angle
        %was greater or less than the next angle.
        if anglePrev > angleNext
            if anglePoint<anglePrev && anglePoint>angleNext
                flagPoint(iPoints)=true;
            else
                flagPoint(iPoints)=false;
            end
        else
            if anglePoint<anglePrev || anglePoint>angleNext
                flagPoint(iPoints)=true;
            else
                flagPoint(iPoints)=false;
            end
        end
    end
end

