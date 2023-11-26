%function [hVal]=graph_heuristic(graphVector,idxX,idxGoal)
%Computes the heuristic  @x   h given by the Euclidean distance between the nodes
%with indexes  @x   idxX and  @x   idxGoal.
%INPUTS
%   graphVector: [NNodesx1] struct 
%   idxX: [1x1] indices in the graphVector to compute the heuristic
%   idxGoal: [1x1] indices in the graphVector to compute the heuristic
%OUTPUS
%   hVal: [1x1] the heuristic Euclidean distance between two elements
function [hVal]=graph_heuristic(graphVector,idxX,idxGoal,varargin)
hVal=norm(graphVector(idxGoal).x-graphVector(idxX).x);
if ~isempty(varargin)
    if varargin{2}
        hVal=norm(mod(graphVector(idxGoal).x+2*pi,2*pi)-mod(graphVector(idxX).x+2*pi,2*pi));
    end
end