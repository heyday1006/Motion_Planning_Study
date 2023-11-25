%This function load the grid structure from 'twolink_testData.mat' and
%convert it to graphVector structure, with or without 'torus'. The A*
%search algorithm is test in three [start;goal] configurations
function twolink_search_test()
flagTorus=false;
if exist(fullfile([pwd '/homework1']),'dir')
    addpath(fullfile([pwd '/homework1']))
end
if exist(fullfile([pwd '/homework2']),'dir')
    addpath(fullfile([pwd '/homework2']))
end
if exist(fullfile([pwd '/homework3']),'dir')
    addpath(fullfile([pwd '/homework3']))
end
load('twolink_testData.mat');
twolink_freeSpaceGraph('torus',flagTorus);
load('twolink_freeSpace_graph','graphVector');
thetaStart_Easy=[0.76;0.12];
thetaGoal_Easy=[0.76;6.00];
thetaStart_Medium=[0.76;0.12];
thetaGoal_Medium=[2.72;5.45];
thetaStart_Hard=[3.3;2.34];
thetaGoal_Hard=[5.49;1.07];
thetaStartList={thetaStart_Easy,thetaStart_Medium,thetaStart_Hard};
thetaGoalList={thetaGoal_Easy,thetaGoal_Medium,thetaGoal_Hard};
nameList={'Easy','Medium','Hard'};
colorList={'g','c','r','m','b'};
for i=1:length(thetaStartList)
    thetaStart=thetaStartList{i};
    thetaGoal=thetaGoalList{i};
    figure(i)
    subplot('Position',[0.04 0.2 0.45 0.65])
    if flagTorus
        [thetaPath]=twolink_search_startGoal(thetaStart,thetaGoal,'torus',flagTorus);
    else
        [thetaPath]=graph_search_startGoal(graphVector,thetaStart,thetaGoal,'torus',flagTorus);
    end
    graph_plot(graphVector)
    hold on
    plot(thetaPath(1,:),thetaPath(2,:),'-.go','MarkerFaceColor','g','MarkerEdgeColor','g','MarkerSize',3)
    hold on
    plot(thetaStart(1),thetaStart(2),'diamond','MarkerFaceColor','m','MarkerEdgeColor','m','MarkerSize',8)
    hold on
    plot(thetaGoal(1),thetaGoal(2),'diamond','MarkerFaceColor','b','MarkerEdgeColor','b','MarkerSize',8)
    title(['In Theta'])
    hold off
    subplot('Position',[0.52 0.2 0.45 0.65])
    %[thetaPath]=graph_search_startGoal(graphVector,thetaStart,thetaGoal);
    %[thetaPath]=twolink_search_startGoal(thetaStart,thetaGoal);
    [xGoal,~,~]=twolink_kinematicMap(thetaGoal);
    [xStart,~,~]=twolink_kinematicMap(thetaStart);
    plot(obstaclePoints(1,:),obstaclePoints(2,:),'rx','LineWidth',3,'MarkerSize',3);
    hold on
    plot(xStart(1),xStart(2),'diamond','MarkerFaceColor','m','MarkerEdgeColor','m','MarkerSize',8)
    hold on
    plot(xGoal(1),xGoal(2),'diamond','MarkerFaceColor','b','MarkerEdgeColor','b','MarkerSize',8)
    hold on
    twolink_plotAnimate(thetaPath, 25);
    hold on
    for iPath=1:size(thetaPath,2)
        iXPath=twolink_kinematicMap(thetaPath(:,iPath));
        plot(iXPath(1),iXPath(2),'-ko','LineWidth',1,'MarkerSize',2);
        hold on
    end
    hold on
    plot(obstaclePoints(1,:),obstaclePoints(2,:),'rx','LineWidth',3,'MarkerSize',3);
    hold on
    plot(xStart(1),xStart(2),'diamond','MarkerFaceColor','m','MarkerEdgeColor','m','MarkerSize',8)
    hold on
    plot(xGoal(1),xGoal(2),'diamond','MarkerFaceColor','b','MarkerEdgeColor','b','MarkerSize',8)
    hold on
    title(['In Eulidean'])
    hold off
    sgtitle([nameList{i} ' Torus:' num2str(flagTorus)])
end
