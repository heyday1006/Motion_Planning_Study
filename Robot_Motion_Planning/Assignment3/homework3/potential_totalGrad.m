%function [gradU]=potential_totalGrad(xEval,world,potential)
%Compute the gradient of the total potential, $ U= U_ attr+  @ @ _i U_ rep,i$,
%where $ $ is given by the variable  @x   potential.repulsiveWeight
%INPUT:     xEval [2x1]:location at x to be evaluated
%           potential [1x1]struct: fields xGoal,shape,repulsiveWeight --
%                     passed to potential_attractive()
%OUTPUT:    gradUEval [2x1]:value of gradU evaluated at xEval
function [gradUEval]=potential_totalGrad(xEval,world,potential)
%attractive potential
gradUAttr = potential_attractiveGrad(xEval,potential);
gradURep = zeros(size(xEval));
for iSphere = 1:length(world)
    sphere = world(iSphere);
    %repulsive potential for each sphere
    gradURep = gradURep + potential_repulsiveSphereGrad(xEval,sphere);
end
gradUEval = gradUAttr + potential.repulsiveWeight*gradURep;