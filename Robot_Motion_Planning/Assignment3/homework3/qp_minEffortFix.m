%function [uOpt,deltaOpt]=qp_minEffort(AAttr,bAttr,ABarrier,bBarrier,m)
%Solves the optimization problem in    @  (  eq:CBF \@@italiccorr ) for the given
%parameters.
function [uOpt,deltaOpt]=qp_minEffortFix(AAttr,bAttr,ABarrier,bBarrier,m)
cvx_begin quiet
    variables u(2,1) delta
    minimize pow_pos(norm(u,2),2)+m*delta^2
    subject to
        AAttr*u+bAttr<=delta
        ABarrier*u+bBarrier<=0
cvx_end
uOpt=u;
deltaOpt=delta;

