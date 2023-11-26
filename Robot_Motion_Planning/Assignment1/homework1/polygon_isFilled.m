%function to check if the polygon is winded clockwise or counter-clockwise
%   INPUT:  vertices:[2xnbVertices] defines the position of each vertex
%   OUTPUT: flag: logical true if the polygon is filled in, vice versa
function flag=polygon_isFilled(vertices)
%https://www.element84.com/blog/determining-the-winding-of-a-polygon-given-as-a-set-of-ordered-points
%Schoelace formula
%to find the ordering of the vertices, sum over all the Vertices (x2-x1)(y2+y1)
%if the final value is negative, it is counter-clockwise (filled-in)
sumSchoelace = 0;
nbVertices = size(vertices,2);
for iVertice = 1:nbVertices-1
    sumSchoelace = sumSchoelace ...
        + (vertices(1,iVertice+1)-vertices(1,iVertice))...
        * ((vertices(2,iVertice+1)+vertices(2,iVertice)));
end
sumSchoelace = sumSchoelace ...
        + (vertices(1,1)-vertices(1,nbVertices))...
        * ((vertices(2,1)+vertices(2,nbVertices)));
flag = (sumSchoelace<0);
