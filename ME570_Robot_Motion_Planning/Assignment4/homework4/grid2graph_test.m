function grid2graph_test
%Example from Fig.1 in the assignment
grid.xx=[1;2];
grid.yy=[1;2;3];
grid.F=false(2,3);
grid.F(1,:)=true;
grid.F(2,1)=true;
disp('grid.F')
disp(grid.F)
graphVectorMedium=grid2graph(grid);
graph_plot(graphVectorMedium,'nodelabels',true)
