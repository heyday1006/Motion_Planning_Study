%function [graphVector]=grid2graph(grid)
%The function should return a  graphVector structure described by the inputs. See
%Figure  fig:grid2graph for an example of the expected inputs and outputs.
function [graphVector]=grid2graph(grid,varargin)
flagTorus=false;
%optional parameters
ivarargin=1;
while ivarargin<=length(varargin)
    switch lower(varargin{ivarargin})
        case 'torus'
            ivarargin=ivarargin+1;
            flagTorus=varargin{ivarargin};
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
    ivarargin=ivarargin+1;
end
%make sure values in F are logicals
grid.F=logical(grid.F);

%total number of nodes in the graph
NNodes=sum(grid.F(:));
%initialize stucture
graphVector=repmat(struct('neighbors',[],'neighborsCost',[],'x',[]),NNodes,1);

%grid size
szGrid=size(grid.F);

%this array keeps, for each non-false element of grid.F, the
%corresponding index in graphVector
%Note that to get the right ordering w.r.t. the definition of grid given in
%HW2, we need use transposes
idxMap=zeros(fliplr(size(grid.F)));
idxMap(grid.F')=1:NNodes;
idxMap=idxMap';

%fill in the x and neighbors fields
for iXCoord=1:szGrid(1)
    for iYCoord=1:szGrid(2)
        if grid.F(iXCoord,iYCoord)
            idxGraph=idxMap(iXCoord,iYCoord);
            graphVector(idxGraph).x=[grid.xx(iXCoord);grid.yy(iYCoord)];
            idxNeighborsF=grid2graph_computeNeighbors(iXCoord,iYCoord,grid.F,'torus',flagTorus);
            graphVector(idxGraph).neighbors=idxMap(sub2ind(szGrid,idxNeighborsF(1,:),idxNeighborsF(2,:)))';
        end
    end
end

%fill in the neighborcost field
%Note: we cannot do this in the previous loop because we need to have all
%the x fields populated first

for iNode=1:NNodes
    if flagTorus
        graphVector(iNode).neighborsCost=...
        sqrt(euclideanDistMatrix(mod(graphVector(iNode).x+2*pi,2*pi),...
        mod([graphVector(graphVector(iNode).neighbors).x]+2*pi,2*pi)))';
    else
        graphVector(iNode).neighborsCost=...
            sqrt(euclideanDistMatrix(graphVector(iNode).x,...
            [graphVector(graphVector(iNode).neighbors).x]))';
    end
end

