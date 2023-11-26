function qp_test()
AAttr = [1 1];
ABarrier = eye(2);
for iCase = 1:4
    [bAttr,bBarrier,mPenalty] = set_parameters(iCase);
    [uOpt,DeltaOpt]=qp_minEffort(AAttr,bAttr,ABarrier,bBarrier,mPenalty);
    fprintf('Case %d: the optimal u is [%.2f,%.2f] and delta is %.2f \n',...
        iCase,uOpt(1),uOpt(2),DeltaOpt);
%   %use Matlab toolbox to check results
%     H = eye(3);
%     H(3,3) = mPenalty;
%     A = [AAttr 1; ABarrier [0;0]];
%     b = [-bAttr;-bBarrier];
%     [x,fval,exitflag,output,lambda] = ...
%    quadprog(H,[0;0;0],A,b);
%     [x' fval] 
end
end
function [bAttr,bBarrier,mPenalty] = set_parameters(iCase)
    switch iCase
        case 1
            bAttr = 1;
            bBarrier = [1;1];
            mPenalty = 1;
        case 2
            bAttr = 1;
            bBarrier = -[1;1];
            mPenalty = 1;
        case 3
            bAttr = -1;
            bBarrier = -[1;1];
            mPenalty = 1;
        case 4
            bAttr = -1;
            bBarrier = -[1;1];
            mPenalty = 100;
    end
end