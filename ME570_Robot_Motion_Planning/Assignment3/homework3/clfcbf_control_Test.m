% epsilon: 1e-3 - 1e-2
% potential.shape: 'conic' or 'quadratic'
% potential.repulsiveWeight: 0.01-0.1
plannerParameters.control = @clfcbf_control;
plannerParameters.NSteps = 20;
plannerParameters.epsilon = 0.5; %1e-3 - 1e-2
potential=struct('shape','quadratic','xGoal',[0;0],'repulsiveWeight',10);
potential_planner_runPlot(potential,plannerParameters)