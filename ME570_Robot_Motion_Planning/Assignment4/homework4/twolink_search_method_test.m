%this function compares different update method for the search algorithm
function twolink_search_method_test()
flagTorus=true;
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
tStartData=tic;
twolink_freeSpaceGraph('torus',flagTorus);
load('twolink_freeSpace_graph','graphVector');
tSpanData=toc(tStartData);
thetaStart_Easy=[0.76;0.12];
thetaGoal_Easy=[0.76;6.00];
thetaStart_Medium=[0.76;0.12];
thetaGoal_Medium=[2.72;5.45];
thetaStart_Hard=[3.3;2.34];
thetaGoal_Hard=[5.49;1.07];
thetaStartList={thetaStart_Easy,thetaStart_Medium,thetaStart_Hard};
thetaGoalList={thetaGoal_Easy,thetaGoal_Medium,thetaGoal_Hard};
nameList={'Easy','Medium','Hard'};
methodList={'bfs','greedy','astar'};
colorList={'g','c','r','m','b'};
diary('twolink_search_method.txt')
disp(['flagTorus: ' num2str(flagTorus)])
figure(1)
for i=1:length(thetaStartList)
    disp(['Case: ' nameList{i}])
    thetaStart=thetaStartList{i};
    thetaGoal=thetaGoalList{i};
    for j=1:length(methodList)
        method=methodList{j};
        subplot('Position',[0.04+0.32*(j-1) 0.66-0.32*(i-1) 0.29 0.26])
        tStart=tic;
        if flagTorus
            [thetaPath]=twolink_search_startGoal(thetaStart,thetaGoal,'method',method,'torus',flagTorus);
        else
            [thetaPath]=graph_search_startGoal(graphVector,thetaStart,thetaGoal,'method',method,'torus',flagTorus);
        end
        tSpan=toc(tStart)+tSpanData;
        disp(['Method: ' method ' Computation time:' num2str(tSpan)]);
        graph_plot(graphVector)
        hold on
        plot(thetaPath(1,:),thetaPath(2,:),'-.go','MarkerFaceColor','g','MarkerEdgeColor','g','MarkerSize',3)
        hold on
        plot(thetaStart(1),thetaStart(2),'diamond','MarkerFaceColor','m','MarkerEdgeColor','m','MarkerSize',8)
        hold on
        plot(thetaGoal(1),thetaGoal(2),'diamond','MarkerFaceColor','b','MarkerEdgeColor','b','MarkerSize',8)
        title([nameList{i} ' ' method])
        hold off
        sgtitle(['Torus:' num2str(flagTorus)])
    end
end
diary off
