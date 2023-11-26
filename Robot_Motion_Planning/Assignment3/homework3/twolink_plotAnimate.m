function twolink_plotAnimate(thetaPath, fps)
%function twolink_plotAnimate(thetaPath, fps)
%Draws each iteration of the twolink manipulator and superimposes
%them, at a certain frames per second.
flagHold=ishold();
for iThetaPath = 1:size(thetaPath,2)
    twolink_plot(thetaPath(:,iThetaPath),'g')
    pause(1/fps)
    if iThetaPath==1 && ~flagHold
      hold on
    end
end
if ~flagHold
  hold off
end
