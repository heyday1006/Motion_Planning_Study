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

%initialize grid for U1,U2,U3
NGrid = 33;
gridSize = 2*pi/33;
gridInit.xx=zeros(1,NGrid);
gridInit.yy=zeros(1,NGrid);
gridU1 = gridInit;
gridU2 = gridInit;
gridU3 = gridInit;
%create U1 U2 U3 grid with xx and yy field
U1_x = [0.2 2*pi-0.2];
U1_y = [0.2 2*pi-0.2];
gridU1.xx = U1_x(1):gridSize:U1_x(2);
gridU1.yy = U1_y(1):gridSize:U1_y(2);
U2_x = [0.5 2*pi+0.3];
U2_y = [-0.4,0.5];
gridU2.xx = U2_x(1):gridSize:U2_x(2);
gridU2.yy = U2_y(1):gridSize:U2_y(2);
U3_x = [-0.4,0.6];
U3_y = [0.4,2*pi+0.3];
gridU3.xx = U3_x(1):gridSize:U3_x(2);
gridU3.yy = U3_y(1):gridSize:U3_y(2);
%assign U1 U2 U3 grid with the F field
gridU1=grid_eval(gridU1,@torus_phi);
gridU2=grid_eval(gridU2,@torus_phi);
gridU3=grid_eval(gridU3,@torus_phi);
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
surf(gridU1.F(:,:,1),gridU1.F(:,:,2),gridU1.F(:,:,3),'FaceColor','cyan','FaceAlpha',0.3)
hold on
surf(gridU2.F(:,:,1),gridU2.F(:,:,2),gridU2.F(:,:,3),'FaceColor','yellow','EdgeColor','none')
hold on
surf(gridU3.F(:,:,1),gridU3.F(:,:,2),gridU3.F(:,:,3),'FaceColor','green','EdgeColor','none')
axis equal
title('torus')
hold off

