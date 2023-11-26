%function [idxExpand]=graph_getExpandList(graphVector,idxNBest,idxClosed)
%Finds the neighbors of element  @x   idxNBest that are not in  @x   idxClosed
%(line  it:expansion in Algorithm  alg:astar).
%Ensure that the vector  @x   idxBest is not included in the list of neighbors
%(i.e., avoid self-loop edges in the graph).
%INPUTS
%   graphVecotr: [NNodesx1]struct 
%   idxNBest: [1x1] index of the element of graphVector of which to find
%             neighbors
%   idxClosed: [NClosedx1] array of indices of elements of graphVector that
%              has been expanded during the search
%OUTPUTS
%   idxExpand: [NNeighborsNotClosedx1] array of indices of neighbors of
%              elment idxNBest in graphVector that are not in idxClosed and not in
%              idxNBest
function [idxExpand]=graph_getExpandList(graphVector,idxNBest,idxClosed)
idxNeighbors=graphVector(idxNBest).neighbors;
idxNeighbors=reshape(idxNeighbors,[],1);
idxClosed=reshape(idxClosed,[],1);
idxExpand=setdiff(idxNeighbors,[idxNBest;idxClosed]);
%idxExpand=idxNeighbors(~ismember(idxNeighbors,[idxNBest;idxClosed]));
