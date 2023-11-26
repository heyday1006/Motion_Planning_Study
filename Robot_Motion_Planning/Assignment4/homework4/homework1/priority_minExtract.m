%function [pQueue,key,cost]=priority_minExtract(pQueue)
%Extract the element with minimum cost from the queue.
%   INPUT:  pQueue: [nbElementx1] structure containing fields key 
%   OUTPU:  pQueue: [(nbElement-1)x1] orignial pQueue without minimum cost
%                   elment, if nbElements=1/0, gives an empty structure
%           key:    identifier with minimum cost
%           cost:   mimimum cost value
function [pQueue,key,cost]=priority_minExtract(pQueue)
nbElements = numel(pQueue);
if nbElements == 0
    key = [];
    cost = [];
else
    [~,idx] = min([pQueue.cost]);
    key = pQueue(idx).key;
    cost = pQueue(idx).cost;
    pQueue = [pQueue(1:idx-1,1); pQueue(idx+1:end,1)];
end
