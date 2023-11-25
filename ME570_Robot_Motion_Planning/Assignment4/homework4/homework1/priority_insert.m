%function [pQueue]=priority_insert(pQueue,key,cost)
%Add an element to the queue.
%   INPUT:  pQueue: [nbElementx1] structure containing fields key and value
%           key:    idendifier for the inserted element
%           cost:   cost associated with the inserted element
%   OUTPU:  pQueue: [(nbElement+1)x1]
function [pQueue]=priority_insert(pQueue,key,cost)
pQueue(end+1,1) = struct('key',key,'cost',cost);
