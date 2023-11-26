%function [vertices1,vertices2]=twolink_polygons()
%Returns the vertices of two polygons that represent the links in a simple 2-D
%two-link manipulator.
function [vertices1,vertices2]=twolink_polygons()
vertices1=[0 -1.11;5 -0.511]';
vertices1=addYReflection(vertices1);
%vertices2=[0 -0.47; 3.47 -0.5; 3.67 -0.75; 4.88 -0.97; 5.11 -0.5; 4 -0.313]';
vertices2=[0 -0.47; 3.97 -0.5; 4.17 -0.75; 5.38 -0.97; 5.61 -0.5; 4.5 -0.313]';
vertices2=addYReflection(vertices2);


function vertices=addYReflection(vertices)
vertices=[vertices fliplr(diag([1 -1])*vertices)];

