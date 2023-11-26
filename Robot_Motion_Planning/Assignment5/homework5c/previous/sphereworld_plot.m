%function sphereworld_plot(world,xGoal)
%Uses sphere_draw to draw the spherical obstacles together with a  * marker at
%the goal location.
function sphereworld_plot(world,xGoal)
nbSpheres=size(world,2);
for iSphere=1:nbSpheres
    sphere_plot(world(iSphere),'b');
    hold on
end
    
plot(xGoal(1,:),xGoal(2,:),'r*');
axis equal
