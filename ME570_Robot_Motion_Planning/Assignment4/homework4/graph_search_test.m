%This function test A* search algorithm and visualize the results of
%'graph_testDat.mat' and a custom world
function graph_search_test()
if exist(fullfile([pwd '/homework1']),'dir')
    addpath(fullfile([pwd '/homework1']))
end
if exist(fullfile([pwd '/homework2']),'dir')
    addpath(fullfile([pwd '/homework2']))
end
if exist(fullfile([pwd '/homework3']),'dir')
    addpath(fullfile([pwd '/homework3']))
end
load('graph_testData','graphVector','graphVector_solved',...
    'graphVectorMedium','graphVectorMedium_solved','pqOpenMedium','closedMedium')
%% graphVector
figure(1)
subplot(1,2,1)
graph_plot(graphVector_solved,...
    'nodeLabels',true,'edgeWeights',true,...
    'backpointerCosts',true,...
    'start',4,'goal',2)
method = 'astar';
idxStart=4;
idxGoal=2;
[xPath,graphVector_solved_my]=graph_search(graphVector,idxStart,idxGoal,'method',method);
title('Provided')
subplot(1,2,2)
plot(xPath(1,:),xPath(2,:),'--mo',...
    'LineWidth',3,...
    'MarkerSize',15,...
    'MarkerEdgeColor','b')%,...'MarkerFaceColor',[0.5,0.5,0.5]
hold on
graph_plot(graphVector_solved_my,...
    'nodeLabels',true,'edgeWeights',true,...
    'backpointerCosts',true,...
    'start',4,'goal',2)
title('Coded')
hold off

%% graphVectorMedium
figure(2)
subplot(1,2,1)
graph_plot(graphVectorMedium_solved,...
    'nodeLabels',true,'backpointerCosts',true,...
    'start',1,'goal',15,...
    'closed',closedMedium,'pqOpen',pqOpenMedium,...
    'best',3,'neighbors',[4 6])
xlim([0.5 6])
ylim([0.5 4.5])
title('Provided')
%axis equal
idxStart=1;
idxGoal=15;
[xPath,graphVectorMedium_solved_my]=graph_search(graphVectorMedium,idxStart,idxGoal,'method',method);
subplot(1,2,2)
plot(xPath(1,:),xPath(2,:),'--mo',...
    'LineWidth',3,...
    'MarkerSize',5,...
    'MarkerEdgeColor','b')%,...'MarkerFaceColor',[0.5,0.5,0.5]
hold on
graph_plot(graphVectorMedium_solved_my,...
    'nodeLabels',true,...
    'backpointerCosts',true,...
    'start',idxStart,'goal',idxGoal,...
    'best',3,'neighbors',[4 6])
%axis equal
xlim([0.5 6])
ylim([0.5 4.5])
title('Coded')
hold off

%% %custom world
grid.xx=linspace(1,10,10);
grid.yy=linspace(1,8,8);
grid.F=true(10,8);
grid.F(2,3)=false;
grid.F(2,1)=false;
grid.F(5,3)=false;
grid.F(8,8)=false;
graphVector=grid2graph(grid);
idxStart=1;
idxGoal=76;
varargin = 'astar';
[xPath,graphVector_solved]=graph_search(graphVector,idxStart,idxGoal,'method',method);

figure(3)
plot(xPath(1,:),xPath(2,:),'--mo',...
    'LineWidth',3,...
    'MarkerSize',15,...
    'MarkerEdgeColor','b')%,...'MarkerFaceColor',[0.5,0.5,0.5]
hold on
graph_plot(graphVector_solved,...
    'nodeLabels',true,...
    'backpointerCosts',true,...
    'start',idxStart,'goal',idxGoal)
hold off




