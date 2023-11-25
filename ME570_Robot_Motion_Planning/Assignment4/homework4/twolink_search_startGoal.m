%This function return the path from A* search algorithm
%INPUTS
%   thetaStart: start location
%   thetaGoal: goal location
%   optional: 'method',method,'torus',torusFlag, where method in
%              {'bfs','greedy','astar'}, torusFlag in {true,false}
%OUTPUTS
%   thetaPath: [2xNPath] coordinates of the points obtained with the traversal
%           of the backpointers (reverse order)
function [thetaPath]=twolink_search_startGoal(thetaStart,thetaGoal,varargin)
load('twolink_freeSpace_graph')
% graph_plot(graphVector)
%identify indices of idxStart,idxGoal in graphVector
idxStart=graph_nearestNeighbors(graphVector,thetaStart,1);
idxGoal=graph_nearestNeighbors(graphVector,thetaGoal,1);
[thetaPath,~]=graph_search(graphVector,idxStart,idxGoal,varargin{:});
thetaPath=[thetaStart thetaPath thetaGoal];