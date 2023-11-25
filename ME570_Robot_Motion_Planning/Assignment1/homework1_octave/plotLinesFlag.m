%This function is to plot the lines specified in pointsStart and pointEnd
%with color specified in flags
%input: pointsStart: [2xnbPoints]
%       pointsEnd: [2xnbPoints]
%       flags:logical [1xnbPoints]
function plotLinesFlag(pointsStart,pointsEnd,flags)
nbPoints = numel(flags);
colorMap = containers.Map([true false],{'g', 'r'});
for iPoint = 1:nbPoints
    %if flag is true: plot in red; if false: plot in gree
    x = [pointsStart(1,iPoint) pointsEnd(1,iPoint)];
    y = [pointsStart(2,iPoint) pointsEnd(2,iPoint)];
    plot(x,y,colorMap(double(flags(iPoint))));
    hold on
end