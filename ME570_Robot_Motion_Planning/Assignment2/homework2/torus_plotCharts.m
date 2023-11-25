%function torus_plotCharts()
%For each one of the chart domains $U_i$ from the previous question: enumerate  a
% @x   grid structure with fields  @x   xx and  @x   yy that define a grid of
%regular point in $U_i$. Use  @x   NGrid=33.  the function grid_eval with
%argument  @x   grid and torus_phi.  the surface described by the values in  @x  
%grid.F using the Matlab function surf. enumerate
function torus_plotCharts()

%Use the commands  @x   hold on and  @x   hold off to show all the charts on the
%same figure. To better show the overlap between the charts, you can use
%different colors each one of them, and the  @x     FaceAlpha  ,0.75 option for
%surf.
NGrid = 33;
tol = 1e-5; %a small number due to open set
%create U1 U2 U3 grid with xx and yy field
[U1_thetaPoints]=line_linspace([1;1], [0;0], 0.2+tol, 2*pi-0.2-tol, NGrid);
[U2_thetaPoints]=line_linspace([(2*pi-0.2)/(2*pi);0.9/(2*pi)], [0.5;-0.4], tol, 2*pi-tol, NGrid);
[U3_thetaPoints]=line_linspace([1/(2*pi);(2*pi-0.1)/(2*pi)], [-0.4;0.4], tol, 2*pi-tol, NGrid);
%create U1 U2 U3 grid with xx, yy, F field
% gridU1 = toGrid(U1_thetaPoints);
% gridU2 = toGrid(U2_thetaPoints);
% gridU3 = toGrid(U3_thetaPoints);
f = @toGrid;
gridU1=grid_eval(f(U1_thetaPoints),@torus_phi);
gridU2=grid_eval(f(U2_thetaPoints),@torus_phi);
gridU3=grid_eval(f(U3_thetaPoints),@torus_phi);
figure(1)
% subplot(2,2,1)
% surf(gridU1.F(:,:,1),gridU1.F(:,:,2),gridU1.F(:,:,3),'FaceColor','cyan','FaceAlpha',0.75)
% axis equal
% title('U_1')
% subplot(2,2,2)
% surf(gridU2.F(:,:,1),gridU2.F(:,:,2),gridU2.F(:,:,3),'FaceColor','yellow','FaceAlpha',0.75)
% axis equal
% title('U_2')
% subplot(2,2,3)
% surf(gridU3.F(:,:,1),gridU3.F(:,:,2),gridU3.F(:,:,3),'FaceColor','green','FaceAlpha',0.75)
% axis equal
% title('U_3')
% subplot(2,2,4)
surf(gridU1.F(:,:,1),gridU1.F(:,:,2),gridU1.F(:,:,3),'FaceColor','cyan','FaceAlpha',0.75)
hold on
surf(gridU2.F(:,:,1),gridU2.F(:,:,2),gridU2.F(:,:,3),'FaceColor','yellow','FaceAlpha',1)
hold on
surf(gridU3.F(:,:,1),gridU3.F(:,:,2),gridU3.F(:,:,3),'FaceColor','green','FaceAlpha',0.8)
axis equal
title('torus')
hold off
function grid = toGrid(thetaPoints)
    grid.xx = thetaPoints(1,:);
    grid.yy = thetaPoints(2,:);