%function [R]=rot2d(theta)
%Create a 2-D rotation matrix from the angle theta according to (1)
function [R]=rot2d(theta)
R=[cos(theta) -sin(theta); sin(theta) cos(theta)];
