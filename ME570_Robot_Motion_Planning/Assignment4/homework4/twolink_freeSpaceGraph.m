%function twolink_freeSpaceGraph()
%The function performs the following steps enumerate  the contents of  @x  
%twolink_freeSpace_data.mat.  grid2graph.  the resulting  @x   vectorGraph struct
%array in the file  @x   twolink_freeSpace_graph.mat. enumerate
function twolink_freeSpaceGraph(varargin)
grid=struct("xx",[],"yy",[],"F",[]);
load('twolink_freeSpace_data')
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
%to graph structure
[graphVector]=grid2graph(grid,'torus',flagTorus);
save('twolink_freeSpace_graph','graphVector');
%graph_plot(graphVector,'nodelabels',false)
