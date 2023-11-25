%function [pQueue]=priority_prepare()
%Create an empty structure array for the queue.
function [pQueue]=priority_prepare()
pQueue=repmat(struct('key',[],'cost',[]),0,1);

