%function [uOpt]=clfcbf_control(xEval,world,potential)
%Compute $u^*$ according to    @  (  eq:clfcbf-qp \@@italiccorr ), where $m$ is
%given by the variable @boxIvory2 potential.repulsiveWeight
%INPUT:     xEval [2x1]:location at x to be evaluated
%           world [NSpheresx1]
%           potential [1x1]struct: fields xGoal,shape,repulsiveWeight --
%                     passed to potential_attractive()
%OUTPUT:    uOpt:solution of QP
function [uOpt]=clfcbf_control(xEval,world,potential)
%initialize
AAttr = zeros(1,2);
bAttr = zeros(1,1);
ABarrier = zeros(size(world,2),2);
bBarrier = zeros(size(world,2),1);
%assign values
AAttr(1,:) = potential_attractiveGrad(xEval,potential)';
bAttr(1,:) = potential_attractive(xEval,potential);
for iSphere = 1:size(world,2)
    ABarrier(iSphere,:) = -sphere_distanceGrad(world(iSphere),xEval)';
    bBarrier(iSphere) = -sphere_distance(world(iSphere),xEval);
end
weight = potential.repulsiveWeight;
[uOpt,deltaOpt]=qp_minEffortFix(AAttr,bAttr,ABarrier,bBarrier,weight);
end