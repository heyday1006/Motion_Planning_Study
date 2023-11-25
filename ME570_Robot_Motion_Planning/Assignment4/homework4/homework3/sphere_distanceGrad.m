%function [gradDPointsSphere]=sphere_distanceGrad(sphere,points)
%Computes the gradient of the signed distance between points and the sphere,
%consistently with the definition of sphere_distance.
%INPUT: sphere [1x1]struct
%       points [2xNPoints]: array of 2D points
%OUTPUT:gradDPointsSphere [1xNPoints]:gradient of the distance. if a point
%                         is at the center --> returns zero
function [gradDPointsSphere]=sphere_distanceGrad(sphere,points)
gradDPointsSphere = zeros(2,size(points,2));
dPointsCenter = zeros(1,size(points,2));
for iPoint=1:size(points,2)
    dPointsCenter(iPoint)= norm(points(:,iPoint)-sphere.xCenter); %distance
end
rdPointsCenter = points-sphere.xCenter;         %difference

flagDistNotZero = dPointsCenter~=0;
%filled-in or hollow
radiusSign = sign(sphere.radius);
if flagDistNotZero
    gradDPointsSphere(:,flagDistNotZero) = radiusSign*rdPointsCenter(:,flagDistNotZero)./dPointsCenter(flagDistNotZero);
end
% if sphere.radius>0 %filled spheres
%     gradDPointsSphere(~flagDistZero) = rdPointsCenter(~flagDistZero)./dPointsCenter(~flagDistZero);
% else %hollow sphere
%     gradDPointsSphere(~flagDistZero) = -rdPointsCenter(~flagDistZero)./dPointsCenter(~flagDistZero);
% end   
        