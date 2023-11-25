%function [xPath]=graph_path(graphVector,idxStart,idxGoal)
%This function follows the backpointers from the node with index  @x   idxGoal in
% @x   graphVector to the one with index  @x   idxStart node, and returns the 
%coordinates (not indexes) of the sequence of traversed elements.
%INPUTS
%   graphVecotr: [NNodesx1]struct 
%   idxStart: [1x1] index of the starting vertex of graphVector 
%   idxGoal: [1x1] index of the end vertex of graphVector 
%OUTPUTS
%   xPath: [2xNPath] coordinates of the points obtained with the traversal
%           of the backpointers (reverse order)
function [xPath]=graph_path(graphVector,idxStart,idxEnd)
xPath = [graphVector(idxEnd).x];
% get the backpointer, 
backpointer = graphVector(idxEnd).backpointer;
while backpointer ~= idxStart
    % then get the coordinates of th ebackpointer.
    bp_coordinates = graphVector(backpointer).x;
    % then add those coordinates to xPath
    xPath = [xPath, bp_coordinates];
    backpointer = graphVector(backpointer).backpointer;
end
xPath = [xPath graphVector(idxStart).x];
xPath = fliplr(xPath);