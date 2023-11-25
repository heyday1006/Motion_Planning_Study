%Returns an array with the degree deg(n) of each vertex n in the graph graphVector.
%INPUTS:    graphVector [NNodesx1] struct: describe a graph
%OUTPUTS:   deg [NNodesx1]: no. of neighbors of each element in graphVector
function deg=graph_degree(graphVector)
deg=arrayfun(@(x) length(x.neighbors),graphVector);