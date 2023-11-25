%function [U]=potential_total(xEval,world,potential)
%Compute the function $U=U_ attr+  @ @ _iU_ rep,i$, where $ $ is given by the
%variable  @x   potential.repulsiveWeight
%INPUT:     xEval [2x1]:location at x to be evaluated
%           world [NSpheresx1]
%           potential [1x1]struct: fields xGoal,shape,repulsiveWeight --
%                     passed to potential_attractive()
%OUTPUT:    UEval:value of potential U evaluated at xEval
function [UEval]=potential_total(xEval,world,potential)
%attractive potential
UAttr = potential_attractive(xEval,potential);
URep = 0;
for iSphere = 1:size(world,2)
    sphere = world(iSphere);
    %repulsive potential for each sphere
    URep = URep + potential_repulsiveSphere(xEval,sphere);
end
UEval = UAttr + potential.repulsiveWeight*URep;