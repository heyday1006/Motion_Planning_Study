%function [flag]=edge_isCollision(vertices1,vertices2)
%Returns  true if the two edges intesect.  Note: if the two edges overlap but are
%colinear, or they overlap only at a single endpoint, they are not considered as
%intersecting (i.e., in these cases the function returns  false). If one of the
%two edges has zero length, the function should always return the result that
%edges are non-intersecting.
%INPUT:  vertices1: [2x2] matrix endpoints of an edge
%        vertices2: [2x2] matrix endpoints of an edge
%OUTPUT: flag     : logical true if two edges intersect
function [flag]=edge_isCollision(vertices1,vertices2)
%The function should be able to handle any orientation of the edges (including
%both vertical and horizontal). Note that the ``overlap'' case needs to be
%checked up to floating-point precision.
p1 = vertices1(:,1);
p2 = vertices1(:,2);
q1 = vertices2(:,1);
q2 = vertices2(:,2);
%two edges is intersecting if (p1,p2,q1) and (p1,p2,q2) is in different 
%orientation, and (p1,q1,p2) and (p1,q2,p2) is in different orientation
p1p2 = p2 - p1;
p1q1 = q1 - p1;
p1q2 = q2 - p1;
sign1 = ([0;0;1]'*cross([p1p2;0],[p1q1;0]))* ([0;0;1]'*cross([p1p2;0],[p1q2;0]));
q1q2 = q2 - q1;
q1p1 = p1 - q1;
q1p2 = p2 - q1;
sign2 = ([0;0;1]'*cross([q1q2;0],[q1p1;0]))* ([0;0;1]'*cross([q1q2;0],[q1p2;0]));
flag = (sign1<0 && sign2<0);  
