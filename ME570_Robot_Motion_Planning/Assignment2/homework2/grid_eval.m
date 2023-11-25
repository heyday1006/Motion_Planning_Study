%function [grid]=grid_eval(grid,fun)
%This function evaluates the function fun (which should be a function handle) on
%each point defined by the grid.
%INPUT:     grid: struct with field xx,yy corresponding to the horizontal
%                        and verticle lines of the grid
%           fun:  function handle taking a [2x1] vector as input and
%                                  scalar/vector as output
%OUTPUT:    grid: struct with filed xx,yy,F, where F is the evaluation of
%                        fun on each grid point [NGrid,NGrid,k]
function [grid]=grid_eval(grid,fun)

%get dimensions of the grid
szMesh=[length(grid.xx) length(grid.yy)];

%get output size of fun (at the origin, just as a test point)
szOutput=size(fun(zeros(2,1)),1);

%allocate output
funEvaluated=zeros([szMesh szOutput]);

%double for loop to evaluate the function
for iMesh=1:szMesh(1)
    for jMesh=1:szMesh(2)
        funEvaluated(iMesh,jMesh,:)=fun([grid.xx(iMesh);grid.yy(jMesh)]);
    end
end

grid.F=funEvaluated;