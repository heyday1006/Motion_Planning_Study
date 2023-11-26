function phereworld_freeSpace_graph_test
load('sphereworld')
if exist(fullfile([pwd '/homework3']),'dir')
    addpath(fullfile([pwd '/homework3']))
end
NCells=27;
[graphVector]=sphereworld_freeSpace_graph(NCells);
figure
subplot(1,2,1)
sphereworld_plot(world,xGoal)
subplot(1,2,2)
graph_plot(graphVector,'nodelabels',false)
sgtitle(['NCells: ' num2str(NCells)])