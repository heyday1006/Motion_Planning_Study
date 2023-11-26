%function [uOpt,DeltaOpt]=qp_minEffort(AAttr,bAttr,ABarrier,bBarrier,mPenalty)
%Solves the optimization problem in    @  (  eq:CBF \@@italiccorr ) for the given
%parameters.
%   INPUT: AAttr [1x2]; bAttr [1x1] -- parameters of one constraint (delta and u)
%          ABarrier [nbOstaclesx2]; bBarrier [nbObstaclesx1] -- parameters
%                    of nbOstacles constraints (u)
%          mPenalty: weight on cost function
%   OUTPUT:uOpt [2x1]   --optimal u
%          DeltaOpt [1] --optimal delta
function [uOpt,DeltaOpt]=qp_minEffort(AAttr,bAttr,ABarrier,bBarrier,mPenalty)
cvx_begin quiet
    variables u(2) delta(1);
    minimize(pow_pos(norm(u,2),2)+mPenalty*delta(1)*delta(1))
    subject to
        AAttr*u + bAttr + delta <= 0;
        ABarrier*u + bBarrier <= 0;
cvx_end
uOpt = u;
DeltaOpt = delta;
