%function sphereworld_search(NCells)
%enumerate  the variables  @x   xStart,  @x   xGoal from      Marker
%file_provided [sphereworld.mat]  @x   sphereworld.mat  each of the three values
%for  @x   NCells: enumerate  the function sphereworld_freeSpace_graph for the
%given value of  @x   NCell.  each goal in  @x   xGoal: enumerate 
%graph_search_startGoal from every starting location in  @x   xStart to that
%goal.  the world using sphereworld_plot, together with the resulting
%trajectories. enumerate enumerate enumerate
%In total, this function should produce six different images (three choices for 
%@x   NCell times two goals).
function sphereworld_search(NCells)
load('sphereworld')
if exist(fullfile([pwd '/homework3']),'dir')
    addpath(fullfile([pwd '/homework3']))
end
colorList={'g','c','r','m','b'};
figure
for iGoal=1:size(xGoal,2)
    subplot(1,2,iGoal)
    sphereworld_plot(world,xGoal(:,iGoal));
    hold on
    for iStart=1:size(xStart,2)
        graphVector=sphereworld_freeSpace_graph(NCells);
        [xPath]=graph_search_startGoal(graphVector,xStart(:,iStart),xGoal(:,iGoal)); 
        plot(xPath(1,:),xPath(2,:),'Color',colorList{iStart},...
        'LineWidth',2,...
        'Marker','.','MarkerSize',1,...
        'MarkerEdgeColor','k')%,...'MarkerFaceColor',[0.5,0.5,0.5]
        hold on
    end
    title(['xGoal #' num2str(iGoal)])
end
sgtitle(['NCells: ' num2str(NCells)])
        


