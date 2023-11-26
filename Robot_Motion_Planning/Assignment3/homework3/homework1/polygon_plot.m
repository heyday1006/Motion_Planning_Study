%function polygon_plot(vertices,style)
%Draws a closed polygon described by  vertices using the style (e.g., color)
%given by  style.
%	INPUT: vertices: [2xnbVertices] defines the position of each vertex
%          sytle:    a selection of {'r','g','b','w',....} defines the plot
%                    color
function polygon_plot(vertices,style)
%Each edge in the polygon must be an arrow pointing from one vertex to the next.
%In Matlab, use the function quiver to actually perform the drawing. The function
%should  not create a new figure.
% plot filled polygon if the ordering is counter-clockwise, and vice versa
% if polygon_isFilled(vertices)
%     hold on
%     patch(vertices(1,:),vertices(2,:),[225 225 225]/255)
% else 
%     hold on
%     set(gca,'Color', [225 225 225]/255)
%     patch(vertices(1,:),vertices(2,:),'w')
% end
u = [diff(vertices(1,:)) vertices(1,1)-vertices(1,size(vertices,2))];
v = [diff(vertices(2,:)) vertices(2,1)-vertices(2,size(vertices,2))];
quiver(vertices(1,:),vertices(2,:),u,v,0,style,'LineWidth',0.9,'AutoScale','off');
%to test for a square
% figure
% vertices = [0 1 1 0; 0 0 1 1];
% polygon_plot(vertices, 'r')
% hold on
% text(vertices(1,:),vertices(2,:),{"v1","v2","v3","v4"},'VerticalAlignment','bottom','HorizontalAlignment','right')
% title("reversed square plot")
