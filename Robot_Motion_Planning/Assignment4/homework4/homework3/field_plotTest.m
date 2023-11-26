load sphereworld.mat

shapeList={'conic','quadratic'};
for i=1:2
    figure(i)
    subplot('Position',[0.033 0.45 0.3 0.26]);
    potential = struct('shape',shapeList{i},'xGoal',xGoal(:,1),'repulsiveWeight',0.1);
    fHandle = @(xInput) potential_attractive(xInput,potential);
    sphereworld_plot(world,xGoal(:,1))
    field_plotThreshold(fHandle,100)
    colorbar
    set(gca,'ColorScale','log')
    xlim([-10 10])
    ylim([-10 10])
    title([shapeList{i} 'U_{attr}'])
    % %
    subplot('Position',[0.366 0.45 0.3 0.26]);
    iSphere = world(1,1);
    fHandle = @(xInput) potential_repulsiveSphere(xInput,iSphere);
    sphereworld_plot(iSphere,xGoal(:,1))
    NGrid=61;
    grid.xx=linspace(iSphere.xCenter(1)-abs(iSphere.radius)-iSphere.distInfluence,iSphere.xCenter(1)+abs(iSphere.radius)+iSphere.distInfluence,NGrid);
    grid.yy=linspace(iSphere.xCenter(2)-abs(iSphere.radius)-iSphere.distInfluence,iSphere.xCenter(2)+abs(iSphere.radius)+iSphere.distInfluence,NGrid);
    field_plotThreshold(fHandle,100,grid)
    set(gca,'ColorScale','log')
    colorbar
    xlim([-10 10])
    ylim([-10 10])
    title("U_{rep,1}")
    % %
    subplot('Position',[0.699 0.45 0.3 0.26]);
    iSphere = world(1,2);
    fHandle = @(xInput) potential_repulsiveSphere(xInput,iSphere);
    sphereworld_plot(iSphere,xGoal(:,1))
    NGrid=61;
    grid.xx=linspace(iSphere.xCenter(1)-abs(iSphere.radius)-iSphere.distInfluence,iSphere.xCenter(1)+abs(iSphere.radius)+iSphere.distInfluence,NGrid);
    grid.yy=linspace(iSphere.xCenter(2)-abs(iSphere.radius)-iSphere.distInfluence,iSphere.xCenter(2)+abs(iSphere.radius)+iSphere.distInfluence,NGrid);
    field_plotThreshold(fHandle,100,grid)
    set(gca,'ColorScale','log')
    colorbar
    xlim([iSphere.xCenter(1)-abs(iSphere.radius)-iSphere.distInfluence iSphere.xCenter(1)+abs(iSphere.radius)+iSphere.distInfluence])
    ylim([iSphere.xCenter(2)-abs(iSphere.radius)-iSphere.distInfluence iSphere.xCenter(2)+abs(iSphere.radius)+iSphere.distInfluence])
    title("U_{rep,2}")
    % %
    subplot('Position',[0.033 0.1 0.3 0.25]);
    potential = struct('shape',shapeList{i},'xGoal',xGoal(:,1),'repulsiveWeight',0.1);
    fHandle = @(xInput) potential_total(xInput,world,potential);
    sphereworld_plot(world,xGoal(:,1))
    field_plotThreshold(fHandle,100)
    set(gca,'ColorScale','log')
    colorbar
    xlim([-10 10])
    ylim([-10 10])
    title("total potential")
    % %
    subplot('Position',[0.333 0.1 0.3 0.25]);
    iSphere = world(1,1);
    fHandle = @(xInput) potential_repulsiveSphereGrad(xInput,iSphere);
    sphereworld_plot(iSphere,xGoal(:,1))
    NGrid=61;
    grid.xx=linspace(iSphere.xCenter(1)-abs(iSphere.radius)-iSphere.distInfluence,iSphere.xCenter(1)+abs(iSphere.radius)+iSphere.distInfluence,NGrid);
    grid.yy=linspace(iSphere.xCenter(2)-abs(iSphere.radius)-iSphere.distInfluence,iSphere.xCenter(2)+abs(iSphere.radius)+iSphere.distInfluence,NGrid);
    field_plotThreshold(fHandle,100,grid)
    xlim([-10 10])
    ylim([-10 10])
    title("gradU_{rep,1}")
    % %
    subplot('Position',[0.666 0.1 0.3 0.25]);
    iSphere = world(1,2);
    fHandle = @(xInput) potential_repulsiveSphereGrad(xInput,iSphere);
    sphereworld_plot(iSphere,xGoal(:,1))
    NGrid=61;
    grid.xx=linspace(iSphere.xCenter(1)-abs(iSphere.radius)-iSphere.distInfluence,iSphere.xCenter(1)+abs(iSphere.radius)+iSphere.distInfluence,NGrid);
    grid.yy=linspace(iSphere.xCenter(2)-abs(iSphere.radius)-iSphere.distInfluence,iSphere.xCenter(2)+abs(iSphere.radius)+iSphere.distInfluence,NGrid);
    field_plotThreshold(fHandle,100,grid)
    xlim([iSphere.xCenter(1)-abs(iSphere.radius)-iSphere.distInfluence iSphere.xCenter(1)+abs(iSphere.radius)+iSphere.distInfluence])
    ylim([iSphere.xCenter(2)-abs(iSphere.radius)-iSphere.distInfluence iSphere.xCenter(2)+abs(iSphere.radius)+iSphere.distInfluence])
    title("gradU_{rep,2}")
end
figure(3)
subplot(1,2,1)
potential = struct('shape','conic','xGoal',xGoal(:,1),'repulsiveWeight',0.1);
fHandle = @(xInput) potential_attractiveGrad(xInput,potential);
sphereworld_plot(world,xGoal(:,1))
field_plotThreshold(fHandle,100)
xlim([-10 10])
ylim([-10 10])
title(['conic ' 'gradU_{attr}'])
subplot(1,2,2)
potential = struct('shape','quadratic','xGoal',xGoal(:,1),'repulsiveWeight',0.1);
fHandle = @(xInput) potential_attractiveGrad(xInput,potential);
sphereworld_plot(world,xGoal(:,1))
field_plotThreshold(fHandle,100)
xlim([-10 10])
ylim([-10 10])
title(['quadratic ' 'gradU_{attr}'])
