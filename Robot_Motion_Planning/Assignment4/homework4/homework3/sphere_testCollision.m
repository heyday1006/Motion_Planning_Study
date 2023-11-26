function sphere_testCollision()
figure(1)
sphere=struct('xCenter',[0;0],'radius',1,'distInfluence',0);
points=3*rand(2,100)-1.5;
[dPointsSphere]=sphere_distance(sphere,points);
sphere_plot(sphere,[0.8,0.8,0.8])
hold on
plot(points(1,dPointsSphere>0),points(2,dPointsSphere>0),'gx','LineWidth',1)
hold on
plot(points(1,dPointsSphere<0),points(2,dPointsSphere<0),'rx','LineWidth',1)
axis equal
title('filled-in sphere')
figure(2)
sphere=struct('xCenter',[0;0],'radius',-1,'distInfluence',0);
points=3*rand(2,100)-1.5;
[dPointsSphere]=sphere_distance(sphere,points);
sphere_plot(sphere,[0.8,0.8,0.8])
hold on
plot(points(1,dPointsSphere>0),points(2,dPointsSphere>0),'gx','LineWidth',1)
hold on
plot(points(1,dPointsSphere<0),points(2,dPointsSphere<0),'rx','LineWidth',1)
axis equal
title('hollow sphere')