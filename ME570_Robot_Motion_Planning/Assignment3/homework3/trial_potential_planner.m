%function [xPath,UPath]=potential_planner(xStart,world,potential,plannerParameters)
%This function uses a given control field (@boxIvory2 plannerParameters.control)
%to implement a generic potential-based planner with step size @boxIvory2
%plannerParameters.epsilon, and evaluates the cost along the returned path. The
%planner must stop when either the number of steps given by @boxIvory2
%plannerParameters.NSteps is reached, or when the norm of the vector given by
%@boxIvory2 plannerParameters.control is less than $10^-3$ (equivalently,
%@boxIvory2 1e-3).
%INPUT:   xStart [2x1]:starting location
%         world [NSpherex1]struc: describe the osbtacles
%         potential [1x1]struct: describe the potential
%         plannerParameters struct: U-handle to the function for computing
%                           the potential function value; control-handle to
%                           compute the negative gradient of the potential
%                           function; epsilon-step size; NSteps: totol
%                           number of step size
%OUTPUT: xPath [2xNSteps]: sequence of locations generated by the planner
%        UPath [1xNSteps]: values of U evaluated at each point on the path
function [xPath,UPath]=trial_potential_planner(xStart,world,potential,plannerParameters)
%initialize xPath and UPath
xPath = NaN * ones(2,plannerParameters.NSteps);
UPath = NaN * ones(1,plannerParameters.NSteps);
xPath(:,1) = xStart;
UPath(1) = plannerParameters.U(xPath(:,1),world,potential);
for iStep = 1:plannerParameters.NSteps-1
    controlCurrent = plannerParameters.control(xPath(:,iStep),world,potential);
    if norm(controlCurrent) < 1e-3
        break;
    end
    xPath(:,iStep+1) = xPath(:,iStep)...
        +plannerParameters.epsilon*controlCurrent;
    UPath(iStep+1) = plannerParameters.U(xPath(:,iStep+1),world,potential);
end
    