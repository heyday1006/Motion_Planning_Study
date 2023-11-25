%function [flagPoint]=polygon_isSelfOccluded(vertex,vertexPrev,vertexNext,point)
%Given the corner of a polygon, checks whether a given point is self-occluded or
%not by that polygon (i.e., if it is ``inside'' the corner's cone or not). Points
%on boundary (i.e., on one of the sides of the corner) are not considered
%self-occluded.  Note that to check self-occlusion from one vertex, knowing the
%two lines formed with the previous and next vertices (and their ordering) is
%sufficient.
%	INPUT: vertex:       [2x1] defines a line starting from vertex to a
%                        point
%          point:        [2xNTestPoints] defines a line starting from
%                        vertex to a point in the set
%          vertexPrev:   [2x1] the vertex before 'vertex'
%          vertexNext:   [2x1] vertex following 'vertex'
%   OUTPUT:flag:         [1xNTestPoints] each is logical true if the line vertex-point is blocked
%                        due to self-occlusion
function [flagPoint]=polygon_isSelfOccluded(vertex,vertexPrev,vertexNext,point)
%Use the function edge_angle to check the angle between the segment  vertex--
%point and the segments  vertex-- vertexPrev and  vertex-- vertexNext. The
%function returns  NaN if  vertex1 or  vertex2 coincides with  vertex0.
NTestPoints = size(point,2);
flagPoint = false(1,NTestPoints);
for iTestPoint = 1:NTestPoints
    %check if it is overlapped
    edgeAngle1 = edge_angle(vertex, point(:,iTestPoint), vertexPrev);
    edgeAngle2 = edge_angle(vertex, point(:,iTestPoint), vertexNext);
    if ~isnan(edgeAngle1) && ~isnan(edgeAngle2) && (edgeAngle1~=0) && (edgeAngle2~=0)
        edgeAngle3 = edge_angle(vertex, vertexNext, vertexPrev,'unsigned');
        edgeAngle4 = edge_angle(vertex, vertexNext, point(:,iTestPoint),'unsigned');
        % if the angle from vertexNext-vertex-vertexPrev is larger than the
        % angle from vertexNext-vertex-point, it is self-occluded
        if edgeAngle4 < edgeAngle3
            flagPoint(iTestPoint) = true;
        end
    end
end
%to check: polygon_isSelfOccluded([1;2],[0;0],[3;0],[1;1])  
    