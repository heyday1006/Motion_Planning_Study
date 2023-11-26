%function [xPath]=graph_search_startGoal(graphVector,xStart,xGoal)
%This function performs the following operations: enumerate  the two indexes  @x 
% idxStart,  @x   idxGoal in  @x   graphVector that are closest to  @x   xStart
%and  @x   xGoal (using graph_nearestNeighbors twice, see Question 
%q:graph-nearest).  graph_search to find a feasible sequence of points  @x  
%xPath from  @x   idxStart to  @x   idxGoal.   @x   xStart and  @x   xGoal,
%respectively, to the beginning and the end of the array  @x   xPath. enumerate
%INPUTS
%   graphVecotr: [NNodesx1]struct 
%   xStart: start location
%   xGoal: goal location
%   optional: 'method',method, where method in {'bfs','greedy','astar'}
%OUTPUTS
%   xPath: [2xNPath] coordinates of the points obtained with the traversal
%           of the backpointers (reverse order)
function [xPath]=graph_search_startGoal(graphVector,xStart,xGoal,varargin)
%identify indices of idxStart,idxGoal in graphVector
idxStart=graph_nearestNeighbors(graphVector,xStart,1);
idxGoal=graph_nearestNeighbors(graphVector,xGoal,1);
[xPath,~]=graph_search(graphVector,idxStart,idxGoal,varargin{:});
xPath=[xStart xPath xGoal];