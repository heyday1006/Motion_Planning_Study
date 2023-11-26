%function [flag]=sphereworld_isCollision(world,x)
%Checks if the points with coordinates  x is inside any of the spheres in  world.
%This is done by iterating over all spheres, using the function sphere_distance
%from Homework 3 on each one of them, and then combining the results.
%INPUTS:    world: struct
%           x [2xNPoints]: coordinates of sampled points to be tested for
%                          collisions
%OUTPUTS:   flag [1xNPoints]: flag(iPoint) true if x(:,iPoints) is in a
%                             sphere
function [flag]=sphereworld_isCollision(world,x)
nSphere=size(world,2);
NPoints=size(x,2);
flag=false(1,NPoints); %output flag
collisionCounter=zeros(1,NPoints); %for counting collision with each obstacle

for iSphere=1:nSphere
    pointDistance=sphere_distance(world(iSphere),x); %distance of the points from the center of an obstacle
    for iPoint=1:NPoints
        if pointDistance(iPoint)<0 %if the point is inside the obstacle
            collisionCounter(iPoint)=collisionCounter(iPoint)+1; %there is a collision
        end
    end
end

for jPoint=1:NPoints
    if collisionCounter(jPoint)>0 %if there was a collision
        flag(jPoint)=true; 
    end
end


end
