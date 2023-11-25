%function field_plotThreshold(fHandle,threshold)
%The function evaluates the function handle  fHandle on points places on a
%regular grid; at any point where the norm of the result is larger than 
%threshold, the result is thresholded (normalized); the function then uses surf
%or quiver to display the result (depending on whether the output of fHandle is
%one- or two-dimensional).
function field_plotThreshold(fHandle,threshold,grid)
if ~exist('threshold','var')
    threshold=10;
end

if ~exist('grid','var')
    NGrid=61;
    grid.xx=linspace(-10,10,NGrid);
    grid.yy=grid.xx;
end
grid=grid_eval(grid,@(x) clip(fHandle(x),threshold));
%generate x,y coordinates for the mesh
%the transpose is necessary to obtain the coordinates in the same way as
%grid_eval
[xMesh,yMesh]=meshgrid(grid.xx,grid.yy);
xMesh=xMesh';
yMesh=yMesh';
switch size(grid.F,3)
    case 1
        surf(xMesh,yMesh,grid.F)
        view(2)
    case 2
        quiver(xMesh,yMesh,grid.F(:,:,1),grid.F(:,:,2))
    otherwise
        error('The number of dimensions in grid.F must be 1 or 2')
end
     

function vct=clip(vct,threshold)
normVct=norm(vct);
if normVct>threshold
    vct=vct/normVct*threshold;
end
