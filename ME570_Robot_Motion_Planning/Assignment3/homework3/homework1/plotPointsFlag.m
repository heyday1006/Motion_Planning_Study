%This function is to plot the lines specified in pointsStart and pointEnd
%with color specified in flags
%input: points: [2xnbPoints]
%       flags:logical [1xnbPoints]
function plotPointsFlag(points,flags)
nbPoints = numel(flags);
colorMap = containers.Map([true false],{'r', 'g'});
for iPoint = 1:nbPoints
    %if flag is true: plot in red; if false: plot in gree
    plot(points(1,iPoint),points(2,iPoint),'o','MarkerEdgeColor',colorMap(flags(iPoint)),'MarkerFaceColor',colorMap(flags(iPoint)));
    hold on
end