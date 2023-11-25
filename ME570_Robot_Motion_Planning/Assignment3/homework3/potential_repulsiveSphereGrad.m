%function [gradURep]=potential_repulsiveSphereGrad(xEval,sphere)
%Compute the gradient of $U_ rep$ for a single sphere, as given by    @  ( 
%eq:repulsive-gradient \@@italiccorr ).
%INPUT:   xEval [2x1] location x at which to evaluate the total repulsive
%                potential
%         sphere struct
%OUTPUT:  gradURep [2x1] the gradient U_rep evaluated at x
function [gradURep]=potential_repulsiveSphereGrad(xEval,sphere)
%This function must use the outputs of sphere_distanceSphere.
nbXEval=size(xEval,2);
dist=sphere_distance(sphere,xEval);
distGrad=sphere_distanceGrad(sphere,xEval);
%Initialize returned potential gradient with a default value
gradURep=zeros(2,nbXEval);
%Set points inside the obstacle
flagDistPositive=dist>0;
gradURep(:,~flagDistPositive)=NaN;
%Set points inside the radius of influence
flagDistInfluence=dist<sphere.distInfluence;
flagValid=flagDistPositive & flagDistInfluence;
if flagValid
    gradURep(:,flagValid)=...
            -((1./dist(flagValid)-1/sphere.distInfluence)...
            .*(1./dist(flagValid)).^2)...
            .*distGrad(:,flagValid);
end