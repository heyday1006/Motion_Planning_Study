%function twolink_planner_runPlot(shape,repulsiveWeight,epsilon)
%This function performs the same steps as potential_planner_test in Questino 
%q:potentialPlannerTest, except for the following: itemize  step  it:grad-handle:
% @x   gradHandle should be set to  @x   @twolink_totalGrad.  step 
%it:grad-handle: Use the contents of the variable  @x   thetaStart instead of  @x
%  xStart to initialize the planner.  step  it:plot-plan: Use twolink_plot to
%plot the results of the planner (since the output  @x   xPath from
%potential_planner will really contain a sequence of join angles, rather than a
%sequence of 2-D points). itemize
function twolink_planner_runPlot(potential,plannerParameters)
%load world data
load('sphereworld.mat','xGoal','thetaStart','world');
%setup fHandles struct
plannerParameters.U=@twolink_potential_total;
plannerParameters.control=@(x,w,p) -twolink_potential_totalGrad(x,w,p);

nbGoals=size(xGoal,2);
nbStarts=size(thetaStart,2);
figNb=0;
%iterate over goals
for iGoal=2
    %finish setup of potential struct
    potential.xGoal = xGoal(:,iGoal);
    %iterate over starts
    for iStart=1:nbStarts
        [thetaPath, UPath] = potential_planner(thetaStart(:,iStart), world, potential, plannerParameters);
        figNb=figNb+1;
        figure(figNb)
        subplot(1,2,1)
        plot(1:size(UPath,2), UPath);
        title(sprintf('Goal %d, start %d, potential',iGoal,iStart))
        xlabel('# steps')
        ylabel('U')
        subplot(1,2,2)
        sphereworld_plot(world, xGoal(:,iGoal));
        hold on
        twolink_plotAnimate(thetaPath(:,1:5:end),25)
        hold off
    end
end
