%function [graphVector,pqOpen]=graph_expandElement(graphVector,idxNBest,idxX,idxGoal,pqOpen)
%This function expands the vertex with index  @x   idxX (which is a neighbor of
%the one with index  @x   idxNBest) and returns the updated versions of  @x  
%graphVector and  @x   pqOpen.
%This function corresponds to lines  it:expansion-start-- it:expansion-end in
%Algorithm  alg:astar.
%INPUTS
%   graphVecotr: [NNodesx1]struct 
%   idxNBest: [1x1] index of the vertex in graphVector that has been popped
%              from the queue
%   idxX: [1x1] neighbor of idxNBest underconsideration
%   idxGoal: [1x1] goal index
%   pqOpen: [NNodesOpenNewx1] struct priority queue of the open nodes
%OUTPUTS
%   graphVecotr: [NNodesx1]struct same as the input, with g and backpointer
%                 updated with the affected cells
%   pqOpen: [NNodesOpenNewx1] struct same as the input, updated with new
%           nodes that has been opened
function [graphVector,pqOpen]=graph_expandElement(graphVector,idxNBest,idxX,idxGoal,pqOpen,varargin)
flagMethod=[];
flagTorus=false;
%optional parameters
ivarargin=1;
while ivarargin<=size(varargin,2)
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
    case 'astar' %'astar' or undefined
        f_n = @(graphVector,idxX,idxGoal)graphVector(idxX).g+graph_heuristic(graphVector,idxX,idxGoal,'torus',flagTorus);
end
g_n = @(graphVector,idxNBest)graphVector(idxNBest).g;
c_n = @(graphVector,idxNBest,idxThis)graphVector(idxNBest).neighborsCost(idxThis);
idxThis=find(ismember(graphVector(idxNBest).neighbors,idxX));
if ~priority_isMember(pqOpen,idxX)
    graphVector(idxX).g=g_n(graphVector,idxNBest)+c_n(graphVector,idxNBest,idxThis);
    graphVector(idxX).backpointer = idxNBest;
    %update pqOpen
    pqOpen = priority_insert(pqOpen,idxX,f_n(graphVector,idxX,idxGoal));
elseif g_n(graphVector,idxNBest)+c_n(graphVector,idxNBest,idxThis)<g_n(graphVector,idxX)
    graphVector(idxX).g=g_n(graphVector,idxNBest)+c_n(graphVector,idxNBest,idxThis);
    graphVector(idxX).backpointer=idxNBest;
end

