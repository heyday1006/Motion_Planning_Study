load('sphereworld.mat');
setList={{'conic',0.1},{'quadratic',0.1},{'quadratic',50}};

for i=1:3
    figure(i)
    parameter=setList{i};
    subplot(1,2,1);
    potential = struct('shape',parameter{1},'xGoal',xGoal(:,2),'repulsiveWeight',parameter{2});
    fHandle = @(xInput) potential_total(xInput,world,potential);
    sphereworld_plot(world,xGoal(:,2))
    field_plotThreshold(fHandle,100)
    set(gca,'ColorScale','log')
    colorbar
    xlim([-10 10])
    ylim([-10 10])
    title("Potential")
    subplot(1,2,2);
    potential = struct('shape',parameter{1},'xGoal',xGoal(:,2),'repulsiveWeight',parameter{2});
    fHandle = @(xInput) potential_totalGrad(xInput,world,potential);
    sphereworld_plot(world,xGoal(:,2))
    field_plotThreshold(fHandle,100)
    xlim([-10 10])
    ylim([-10 10])
    title("Graident")
end
