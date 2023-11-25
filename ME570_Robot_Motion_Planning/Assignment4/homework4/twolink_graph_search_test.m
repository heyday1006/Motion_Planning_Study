function twolink_graph_search_test()
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
load('twolink_freeSpace_data')
tStartData=tic;
%total number of nodes in the graph
NNodes=sum(grid.F(:));
%initialize stucture
graphVector=repmat(struct('neighbors',[],'neighborsCost',[],'x',[]),NNodes,1);
szGrid=size(grid.F);
idxMap=zeros(fliplr(size(grid.F)));
idxMap(grid.F')=1:NNodes;
idxMap=idxMap';
for iXCoord=1:szGrid(1)
    for iYCoord=1:szGrid(2)
        if grid.F(iXCoord,iYCoord)
            idxGraph=idxMap(iXCoord,iYCoord);
            graphVector(idxGraph).x=[grid.xx(iXCoord);grid.yy(iYCoord)];
        end
    end
end
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
diary('twolink_graph_search.txt')
disp(['flagTorus: ' num2str(flagTorus)])
for i=1:length(thetaStartList)
    disp(['Case: ' nameList{i}])
    thetaStart=thetaStartList{i};
    thetaGoal=thetaGoalList{i};
    for j=1:length(methodList)
        method=methodList{j};
        tStart=tic;
        idxStart=graph_nearestNeighbors(graphVector,thetaStart,1);
        idxGoal=graph_nearestNeighbors(graphVector,thetaGoal,1);
        [thetaPath,~]=twolink_graph_search(graphVector,idxStart,idxGoal,grid.F,'method',method,'torus',flagTorus);
        thetaPath=[thetaStart thetaPath thetaGoal];
        tSpan=toc(tStart)+tSpanData;
        disp(['Method: ' method ' Computation time:' num2str(tSpan)]);
    end
end
diary off
