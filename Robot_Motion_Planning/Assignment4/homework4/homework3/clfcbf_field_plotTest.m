load('sphereworld.mat');
setList={{'quadratic',1}};
NGrid=10;
grid.xx=linspace(-10,10,NGrid);
grid.yy=linspace(-10,10,NGrid);

for i=1:1
    figure(i)
    potential = struct('shape',parameter{1},'xGoal',xGoal(:,2),'repulsiveWeight',parameter{2});
    fHandle = @(xInput) clfcbf_control(xInput,world,potential);
    sphereworld_plot(world,xGoal(:,2))
    field_plotThreshold(fHandle,100,grid)
    xlim([-10 10])
    ylim([-10 10])
    title("u^*(x)")
end
