%function [URep]=potential_repulsiveSphere(xEval,sphere)
%Evaluate the sum repulsive potentials from  sphere at the location $x= xEval$.
%The function returns the repulsive potential as given by    @  (  eq:repulsive
%\@@italiccorr ).
function [URep]=potential_repulsiveSphere(xEval,sphere)
nbXEval=size(xEval,2);
dist=sphere_distance(sphere,xEval);
%Initialize returned potential with a default value
URep=zeros(1,nbXEval);
%Set points inside the obstacle
flagDistPositive=dist>0;
URep(~flagDistPositive)=NaN;
%Set points inside the radius of influence
flagDistInfluence=dist<sphere.distInfluence;
flagValid=flagDistPositive & flagDistInfluence;
URep(flagValid)=(1./dist(flagValid)-1/sphere.distInfluence).^2/2;