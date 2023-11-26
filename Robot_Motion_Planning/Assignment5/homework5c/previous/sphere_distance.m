%function [dPointsSphere]=sphere_distance(sphere,points)
%Computes the signed distance between points and the sphere, while taking into
%account whether the sphere is hollow or filled in.
%INPUT: sphere [1x1]struct
%       points [2xNPoints]: array of 2D points
%OUTPUT:dPointsSphere [1xNPoints]:distance between each point and surface
%                     of the sphere, negative if points is inside the
%                     sphere for filled-in obstacles, outside for hollow
function [dPointsSphere]=sphere_distance(sphere,points)
%Remember that the radius of the sphere is negative for hollow spheres.
dPointsCenter = zeros(1,size(points,2));
for iPoint=1:size(points,2)
    dPointsCenter(iPoint)= norm(points(:,iPoint)-sphere.xCenter); %distance
end
%filled-in or hollow
radiusSign = sign(sphere.radius);
dPointsSphere = radiusSign*dPointsCenter - sphere.radius;
% if sphere.radius>0 %filled spheres
%     dPointsSphere = dPointsCenter - sphere.radius;
% else %hollow sphere
%     dPointsSphere = abs(sphere.radius) - dPointsCenter;
% end
    