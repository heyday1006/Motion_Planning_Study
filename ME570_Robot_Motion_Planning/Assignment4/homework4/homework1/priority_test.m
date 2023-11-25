function priority_test()
% diary("matlabDiary.txt")
% diary on
%initialize an empty queue
[pQueue]=priority_prepare();
fprintf('empty queue with key: %s and cost: %s\n', '[]','[]');
%add three elemets to the queue
i = 1;
pQueue = priority_insert(pQueue,'a',5);
fprintf('new element with key: %s and cost: %d\n', (pQueue(i).key),(pQueue(i).cost));
pQueue = priority_insert(pQueue,true,10);
i = i + 1;
fprintf('new element with key: %s and cost: %d\n', 'true',(pQueue(i).cost));
pQueue = priority_insert(pQueue,'c',3);
i = i + 1;
fprintf('new element with key: %s and cost: %d\n', (pQueue(i).key),(pQueue(i).cost));
%extract a minimum element
[pQueue,key,cost]=priority_minExtract(pQueue);
i = i - 1;
fprintf('minimum element is with key: %s and cost: %d\n', (key),(cost));
%add another element
pQueue = priority_insert(pQueue,'d',6);
fprintf('new element with key: %s and cost: %d\n',(pQueue(i).key),(pQueue(i).cost));
%check if an element is present
flag1 = priority_isMember(pQueue,'c');
fprintf('is element with key: %s present?   %s\n', 'c', checkflag(flag1));
flag2 = priority_isMember(pQueue,true);
fprintf('is element with key: %s present?   %s\n', 'true', checkflag(flag2));
%remove all elements in the structure array
[pQueue]=priority_prepare();
fprintf('all elements are removed!\n')
fprintf('empty queue with key: %s and cost: %s\n', '[]','[]');
% diary off
end
function output = checkflag(flag)
    if flag == true
        output = 'yes';
    elseif flag == false
        output = 'no';
    end
end