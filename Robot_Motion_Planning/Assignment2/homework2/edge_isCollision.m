%function [flag]=edge_isCollision(vertices1,vertices2)
%Returns  true if the two edges intesect.  Note: if the two edges overlap but are
%colinear, or they overlap only at a single endpoint, they are not considered as
%intersecting (i.e., in these cases the function returns  false). If one of the
%two edges has zero length, the function should always return the result that
%edges are non-intersecting.
function [flag]=edge_isCollision(vertices1,vertices2,varargin)

flagConsiderEndpointsAsCollisions=false;
tolDistance=1e-12;
tolParallel=1e-12;

%optional parameters
ivarargin=1;
while ivarargin<=numel(varargin)
    switch lower(varargin{ivarargin})
        case 'flagconsiderendpointsascollisions'
            ivarargin=ivarargin+1;
            flagConsiderEndpointsAsCollisions=varargin{ivarargin};
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
    ivarargin=ivarargin+1;
end

%check zero-length case
if norm(diff(vertices1,[],2))<tolDistance || norm(diff(vertices2,[],2))<tolDistance
    flag=false;
else
    %make the linear system
    linSysA=[vertices1(:,2)-vertices1(:,1) vertices2(:,1)-vertices2(:,2)];
    linSysb=[vertices2(:,1)-vertices1(:,1)];
    %check the smallest singular value of A to see if they are parallel
    %if the min singular value is close to zero, it means that the linear
    %system does not have a solution, i.e., the lines are parallel
    if min(svd(linSysA))<tolParallel
        flag=false;
    else
        tIntersect=linSysA\linSysb;
        %set lower/upper limits on t depending if we need to consider the
        %endpoints or not
        if flagConsiderEndpointsAsCollisions
            tMin=-tolDistance;
            tMax=1+tolDistance;
        else
            tMin=tolDistance;
            tMax=1-tolDistance;
        end
        flag=all(tIntersect>tMin) & all(tIntersect<tMax);
    end
end
