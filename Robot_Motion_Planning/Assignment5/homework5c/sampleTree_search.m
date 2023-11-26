%function [xPath,graphVector]=sampleTree_search(world,xStart,xEnd,goalDistThreshold)
%Implement a tree-based sampling search algorithm. The function performs the
%following steps: enumerate  a @boxIvory2 graphVector containing only a vertex at
%@boxIvory2 xStart. it:extend Call sampleTree_extend to grow the tree with a new
%sample with location @boxIvory2 xSample.  the distance between @boxIvory2
%xSample and @boxIvory2 xEnd is less then @boxIvory2 goalDistThreshold, call
%prm_localPlannerIsCollision to try to connect the goal to the tree: enumerate 
%the connection does  not succeed, repeat from step  it:extend, up to @boxIvory2
%NTrials=1000 times.  the connection succeeds, add @boxIvory2 xEnd to @boxIvory2
%graphVector (with the correct backpointer), and then use graph_path to return
%the path from @boxIvory2 xStart to @boxIvory2 xEnd. enumerate enumerate
function [xPath,graphVector]=sampleTree_search(world,xStart,xEnd,goalDistThreshold)
%initialize the graphVector with xStart
graphVector(1).x=xStart;
graphVector(1).neighbors=[];
graphVector(1).neighborCost=[];
NTrials=1000;
radius=2.6;%goalDistThreshold;
maxDistEdgeCheck=0.1;
xPath=[xStart xEnd];
for iTrial=1:NTrials
    %grow the tree with a new sample xSample
    [graphVector,xSample]=sampleTree_extend(graphVector,radius,world);
    if norm(xEnd-xSample)<goalDistThreshold
        flagIsCollision=prm_localPlannerIsCollision(world,xSample,xEnd,maxDistEdgeCheck);
        if ~flagIsCollision
            graphVector(end).neighbors=size(graphVector,1)+1;
            graphVector(end).neighborCost=norm(xEnd-xSample);
            structTemp=struct('x',xEnd,'backpointer',size(graphVector,1),...
                'neighbors',size(graphVector,1),'neighborCost',norm(xEnd-xSample));
            graphVector=struct_cat(1,graphVector,structTemp);
            [xPath]=graph_path(graphVector,1,size(graphVector,1));
            break;
        end
    end
end
        