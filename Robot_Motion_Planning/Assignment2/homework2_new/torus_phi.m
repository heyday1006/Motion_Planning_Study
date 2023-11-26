%function [xTorus]=torus_phi(theta)
%Implements equation    @  (  eq:chartTorus \@@italiccorr ).
%INPUT:     theta:  [2xNPoints] 
%OUTPUT:    xTorus: [3xNPoints] embedded theta into 3D space
function [xTorus]=torus_phi(theta)
xTorus = zeros(3,size(theta,2));
for iTheta = 1:size(theta,2)
    x_circle = rot2d(theta(1,iTheta)) * [1;0];
    sumTemp = [1 0;0 0;0 1]*x_circle + [3;0;0];
    xTorus(:,iTheta) = blkdiag(rot2d(theta(2,iTheta)),1) * sumTemp ;
end
