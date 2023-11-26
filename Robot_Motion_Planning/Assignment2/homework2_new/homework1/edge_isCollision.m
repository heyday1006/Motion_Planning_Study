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
edgeP1 = vertices1(:,1);
edgeP2 = vertices1(:,2);
edgeQ1 = vertices2(:,1);
edgeQ2 = vertices2(:,2);
%two edges is intersecting if (edgeP1,edgeP2,edgeQ1) and (edgeP1,edgeP2,edgeQ2) is in different 
%orientation, and (edgeP1,edgeQ1,edgeP2) and (edgeP1,edgeQ2,edgeP2) is in different orientation
edgeP1P2 = edgeP2 - edgeP1;
edgeP1Q1 = edgeQ1 - edgeP1;
edgeP1Q2 = edgeQ2 - edgeP1;
sign1 = ([0;0;1]'*cross([edgeP1P2;0],[edgeP1Q1;0]))* ([0;0;1]'*cross([edgeP1P2;0],[edgeP1Q2;0]));
edgeQ1Q2 = edgeQ2 - edgeQ1;
edgeQ1P1 = edgeP1 - edgeQ1;
edgeQ1P2 = edgeP2 - edgeQ1;
sign2 = ([0;0;1]'*cross([edgeQ1Q2;0],[edgeQ1P1;0]))* ([0;0;1]'*cross([edgeQ1Q2;0],[edgeQ1P2;0]));
flag = (sign1<0 && sign2<0);  
