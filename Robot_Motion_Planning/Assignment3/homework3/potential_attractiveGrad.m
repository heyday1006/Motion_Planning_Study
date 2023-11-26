%function [gradUAttr]=potential_attractiveGrad(xEval,potential)
%Evaluate the gradient of the attractive potential $ U_ attr$ at a point  xEval.
%The gradient is given by the formula If  potential.shape is equal to  'conic',
%use $p=1$. If  potential.shape is equal to  'quadratic', use $p=2$.
%INPUT:  xEval [2x1]:location x to be evaluated
%        potential [1x1]struct: potential.xGoal, potential.shape specifies
%                   the shape of attractive potential,
%                   potential.repulsiveWeight
%OUTPUT: gradUAttr [2x1]
function [gradUAttr]=potential_attractiveGrad(xEval,potential)
switch lower(potential.shape)
    case 'conic'
        pval=1;
    case 'quadratic'
        pval=2;
    otherwise
        error('Attractive potential shape not recognized')
end
gradUAttr=pval*norm(xEval-potential.xGoal)^(pval-2)*(xEval-potential.xGoal);
% if isequal(xEval,potential.xGoal) && isequal(potential.shape,'conic')
%     gradUAttr=NaN;
% end
