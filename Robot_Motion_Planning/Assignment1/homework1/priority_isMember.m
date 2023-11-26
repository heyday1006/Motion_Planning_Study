%function [flag]=priority_isMember(pQueue,key)
%Check whether an element with a given key is in the queue or not.
%   INPUT:  pQueue: [nbElementx1] structure containing fields key and value
%           key:    idendifier for element which the presence will be
%                   checked
%   OUTPU:  flag:   logical true if pQueue has the input key
function [flag]=priority_isMember(pQueue,key)
%Remember that the  key argument could be a number, a vector of numbers, a
%string, or any other arbitrary type. As such, you should use the Matlab function
%isequal to check for equality between keys (this function works for arbitrary
%types of variables, run  doc isequal on the Matlab prompt for details).
flag = false;
for iKey = 1:numel(pQueue)
    if isequal(key,pQueue(iKey).key)
        flag = true;
    end
end