%function [xPath,graphVector]=graph_search(graphVector,idxStart,idxGoal)
%Implements the  @x   A* algorithm, as described by the pseudo-code in Algorithm 
%alg:astar.
%Set a maximum limit of iterations in the main  @x   A* loop on line 
%it:astar-main-loop of Algorithm  alg:astar. This will prevent the algorithm from
%remaining stuck on malformed graphs (e.g., graphs containing a node as a
%neighbor of itself), or if you make some mistake during development.
%coordinates (not indexes) of the sequence of traversed elements.
%INPUTS
%   graphVecotr: [NNodesx1]struct 
%   idxStart: [1x1] index of the starting vertex of graphVector 
%   idxGoal: [1x1] index of the end vertex of graphVector 
%   optional: 'method',method,'torus',torusFlag, where method in
%              {'bfs','greedy','astar'}, torusFlag in {true,false}
%OUTPUTS
%   xPath: [2xNPath] coordinates of the points obtained with the traversal
%           of the backpointers (reverse order)
%   graphVecotr: [NNodesx1]struct same as input, populate field backpointer
%                and g by the search
function [xPath,graphVector]=graph_search(graphVector,idxStart,idxGoal,varargin)
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
idxClosed=[];
xPath=[];
iteration=0;
maxIter=100000;
%%algorithm
while ~isempty(pqOpen) && iteration<maxIter
    [pqOpen,idxNBest,costNBest]=priority_minExtract(pqOpen);
    idxClosed(end+1,1)=idxNBest;
    if idxNBest==idxGoal
        xPath=graph_path(graphVector,idxStart,idxGoal);
        break;
    end
    %expand
    [idxExpand]=graph_getExpandList(graphVector,idxNBest,idxClosed);
    for iExpand=1:numel(idxExpand)
        idxX=idxExpand(iExpand);
        [graphVector,pqOpen]=graph_expandElement(graphVector,idxNBest,idxX,idxGoal,pqOpen,varargin{:});
    end
    iteration=iteration+1;
end

