%function [graphVector,xSample]=sampleTree_extend(graphVector,radius,world)
%The function performs the following steps: enumerate  graph_degree to obtain the
%array of degrees @boxIvory2 deg of the nodes in the tree.  an array @boxIvory2
%fDistr such that @boxIvory2 fDistr(i) is proportional to @boxIvory2
%1/(deg(i)+1). it:sampleNode Use rand_discrete to sample a node with index
%@boxIvory2 idxSample according to @boxIvory2 fDistr. it:sampleState Use
%sphereworld_sample with the optional arguments to generate a sample location
%@boxIvory2 xSample around the location of @boxIvory2 idxSample according to a
%Gaussian distribution with variance @boxIvory2 radius.  graph_nearestNeighbors
%(with @boxIvory2 kNeighbors=1) to find the index @boxIvory2 idxNear of the
%single closest neighbor of @boxIvory2 xSample.  prm_localPlannerIsCollision to
%check whether @boxIvory2 xSample can be connected to the closest neighbor. If
%there is no collision, add @boxIvory2 xSample to @boxIvory2 graphVector, setting
%its backpointer to @boxIvory2 idxNear, and updating the @boxIvory2 neighbor and
%@boxIvory2 neighborCost fields of @boxIvory2 graphVector[idxNear].  the steps
%above from  it:sampleNode until a sample is successfully added, or up to 100
%times. enumerate
function [graphVector,xSample]=sampleTree_extend(graphVector,radius,world)
maxDistEdgeCheck=0.1;
%array of number of neighbors of each node
deg=graph_degree(graphVector);
%array of probability 
fDistr=1./(deg+1);
%sampled node
xDistr=(1:size(graphVector,1));
for iRun=1:100
    idxSample=rand_discrete(xDistr,fDistr);
    %a sample location around idxSample with variance:radius
    [xSample]=sphereworld_sample(world,...
        'distribution','gaussian','mean',graphVector(idxSample).x,'size',radius);
    %nearest node index of xSample
    [idxNear]=graph_nearestNeighbors(graphVector,xSample,1);
    %check if xSample can be connected to node idxNear
    flagCollision=prm_localPlannerIsCollision(world,xSample,...
        graphVector(idxNear).x,maxDistEdgeCheck);
    if ~flagCollision
        structTemp=struct('x',xSample,'backpointer',idxNear,...
            'neighbors',idxNear,'neighborCost',norm(graphVector(idxNear).x-xSample));
        graphVector=struct_cat(1,graphVector,structTemp);
        graphVector(idxNear).neighbors=[graphVector(idxNear).neighbors;size(graphVector,1)];
        graphVector(idxNear).neighborCost=...
            [graphVector(idxNear).neighborCost;norm(graphVector(idxNear).x-xSample)];
        break;
    end
end