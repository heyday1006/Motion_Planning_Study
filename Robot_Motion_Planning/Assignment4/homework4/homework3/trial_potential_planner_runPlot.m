%function potential_planner_test(shape,repulsiveWeight,epsilon,fHandleControl)
%This function performs the following steps: enumerate  the problem data from the
%file  sphereworld.mat.  each goal location in  world.xGoal: enumerate  the
%function sphereworld_plot to plot the world in a first figure.  the structure 
%potential.  the function potential_planner with the problem data and the input
%arguments. The function needs to be called five times, using each one of the
%initial locations given in  xStart (also provided in  sphereworld.mat).  each
%call, plot the resulting trajectory superimposed to the world in the first
%window; in a second window, plot  UPath (using the same color). enumerate
%enumerate To avoid too much clutter, use separate pairs of figures for different
%goal locations (but keep all the different paths from the different starting
%points to the same goal location in the same pair of figures).
function trial_potential_planner_runPlot(potential,plannerParameters)
%styles used for each line
styles={'r','g','b','c','m'};
%load world data
load('sphereworld.mat','xGoal','xStart','world');
%setup fHandles struct
plannerParameters.U=@potential_total;
% xStart = xStart(:,5);
nbStarts=size(xStart,2);
%iterate over goals
for iGoal=1:size(xGoal,2)
    figure(iGoal)
    potential.xGoal = xGoal(:,iGoal);
    subplot(1,2,1)
    sphereworld_plot(world, xGoal(:,iGoal));
    %iterate over starts
    for iStart=1:nbStarts
        subplot(1,2,1)
        [xPath, UPath] = trial_potential_planner(xStart(:,iStart), world, potential, plannerParameters);
        hold on
        plot(xPath(1,:), xPath(2,:), [styles{iStart} '.-']);
        hold off
        subplot(1,2,2)
        hold on
        plot(1:size(UPath,2),UPath,styles{iStart})
        hold off
    end
    subplot(1,2,1)
    title(['Goal ' num2str(iGoal) ', trajectories'])
    subplot(1,2,2)
    title(['Goal ' num2str(iGoal) ', potential'])
    xlabel('# steps')
    ylabel('U')
end