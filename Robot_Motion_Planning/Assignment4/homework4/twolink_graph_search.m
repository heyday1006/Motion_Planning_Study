%comparing to A* in graph_search(), there graphVector doesn't have an empty
%neighobors and neighborsCost ahead, instead, only during getExpandList(), it
%finds the nearest neighbors and costs.
%INPUTS
%   graphVecotr: [NNodesx1]struct 
%   idxStart: [1x1] index of the starting vertex of graphVector 
%   idxGoal: [1x1] index of the end vertex of graphVector 
%OUTPUTS
%   xPath: [2xNPath] coordinates of the points obtained with the traversal
%           of the backpointers (reverse order)
%   graphVecotr: [NNodesx1]struct same as input, populate field backpointer
%                and g by the search
function [xPath,graphVector]=twolink_graph_search(graphVector,idxStart,idxGoal,flag,varargin)
flagMethod=[];
flagTorus=false;
%optional parameters
ivarargin=1;
while ivarargin<=length(varargin)
    switch lower(varargin{ivarargin})
        case 'method'
            ivarargin=ivarargin+1;
            flagMethod=varargin{ivarargin};
        case 'torus'
            ivarargin=ivarargin+1;
            flagTorus=varargin{ivarargin};
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
    ivarargin=ivarargin+1;
end
if isempty(flagMethod)
    flagMethod='astar';
end
switch lower(flagMethod)
    case 'bfs'
        f_n = @(graphVector,idxX,idxGoal)graphVector(idxX).g;
    case 'greedy'
        f_n = @(graphVector,idxX,idxGoal)graph_heuristic(graphVector,idxX,idxGoal,'torus',flagTorus);
    case 'astar' 
        f_n = @(graphVector,idxX,idxGoal)graphVector(idxX).g+graph_heuristic(graphVector,idxX,idxGoal,'torus',flagTorus);
end

%% Initialization
graphVector(idxStart).g=0;
graphVector(idxStart).backpointer=[];
%priority queue
pqOpen=priority_prepare();
pqOpen=priority_insert(pqOpen,idxStart,f_n(graphVector,idxStart,idxGoal));
%list of closed edges
idxClosed=[];xPath=[];
iteration=0;
maxIter=100000;
%grid size
szGrid=size(flag);
%fill in the x and neighbors fields
idxMap=zeros(fliplr(size(flag)));
idxMap(flag')=1:length(graphVector);
idxMap=idxMap';
%%algorithm
while ~isempty(pqOpen)&&iteration<maxIter
    [pqOpen,idxNBest,costNBest]=priority_minExtract(pqOpen);
    idxClosed(end+1,1)=idxNBest;
    if idxNBest==idxGoal
        xPath=graph_path(graphVector,idxStart,idxGoal);
        break;
    end
    %expand
    [idxExpand,graphVector]=graph_getExpandList(graphVector,idxNBest,idxClosed,flag,flagTorus,szGrid,idxMap);
    for i=1:length(idxExpand)
        idxX=idxExpand(i);
        [graphVector,pqOpen]=graph_expandElement(graphVector,idxNBest,idxX,idxGoal,pqOpen,varargin{:});
    end
    iteration=iteration+1;
end
function [idxExpand,graphVector]=graph_getExpandList(graphVector,idxNBest,idxClosed,flag,flagTorus,szGrid,idxMap)
if isempty(graphVector(idxNBest).neighbors)
    [iXCoord,iYCoord]=ind2sub(szGrid,find(idxMap==idxNBest));
    idxNeighborsF=grid2graph_computeNeighbors(iXCoord,iYCoord,flag,'torus',flagTorus);
    idxNeighbors=idxMap(sub2ind(szGrid,idxNeighborsF(1,:),idxNeighborsF(2,:)))';
    %[flagTheta]=twolink_isCollision(graphVector(idxNeighbors).x,ind2sub(size(flag),flag(flag==false)));
    %idxNeighbors=idxNeighbors(flagTheta);
    idxExpand=idxNeighbors(~ismember(idxNeighbors,[idxNBest;idxClosed]));
    graphVector(idxNBest).neighbors=idxNeighbors;
    iNode=idxNBest;
    if flagTorus
        graphVector(iNode).neighborsCost=...
        sqrt(euclideanDistMatrix(mod(graphVector(iNode).x+2*pi,2*pi),...
        mod([graphVector(graphVector(iNode).neighbors).x]+2*pi,2*pi)))';
    else
        graphVector(iNode).neighborsCost=...
            sqrt(euclideanDistMatrix(graphVector(iNode).x,...
            [graphVector(graphVector(iNode).neighbors).x]))';
    end
else
    idxNeighbors=graphVector(idxNBest).neighbors;
    idxExpand=idxNeighbors(~ismember(idxNeighbors,[idxNBest;idxClosed]));
end

