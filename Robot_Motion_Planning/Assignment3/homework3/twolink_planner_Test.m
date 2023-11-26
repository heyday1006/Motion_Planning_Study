% epsilon: 1e-3 - 1e-2
% potential.shape: 'conic' or 'quadratic'
% potential.repulsiveWeight: 0.01-0.1
%plannerParameters.control = @(xEval,world,potential)-1*potential_totalGrad(xEval,world,potential);
plannerParameters.NSteps = 400;
plannerParameters.epsilon = 0.0005; %1e-3 - 1e-2
potential=struct('shape','quadratic','xGoal',[0;0],'repulsiveWeight',0.005);
twolink_planner_runPlot(potential,plannerParameters)