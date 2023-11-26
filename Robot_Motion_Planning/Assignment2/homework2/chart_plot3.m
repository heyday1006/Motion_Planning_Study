U1_x = [0.2 2*pi-0.2];
U1_y = [0.2 2*pi-0.2];
vertices1 = transform(U1_x,U1_y);
U2_x1 = [0 0.3];
U2_x2 = [0.5 2*pi];
U2_y1 = [0 0.5];
U2_y2 = [2*pi-0.4 2*pi];
vertices2_1 = transform(U2_x1,U2_y1);
vertices2_2 = transform(U2_x1,U2_y2);
vertices2_3 = transform(U2_x2,U2_y1);
vertices2_4 = transform(U2_x2,U2_y2);
vertices2 = {vertices2_1,vertices2_2,vertices2_3,vertices2_4};
U3_x1 = [0 0.6];
U3_x2 = [2*pi-0.4 2*pi];
U3_y1 = [0 0.3];
U3_y2 = [0.4 2*pi];
vertices3_1 = transform(U3_x1,U3_y1);
vertices3_2 = transform(U3_x1,U3_y2);
vertices3_3 = transform(U3_x2,U3_y1);
vertices3_4 = transform(U3_x2,U3_y2);
vertices3 = {vertices3_1,vertices3_2,vertices3_3,vertices3_4};
U4_x = [0 2*pi];
U4_y = [0 2*pi];
vertices4 = transform(U4_x,U4_y);
figure(1)
subplot(2,2,1)
patch(vertices1(:,1),vertices1(:,2),'green','FaceAlpha',.1)
xticks([0:pi/3:2*pi])
xticklabels({'0','1/3\pi','2/3\pi','\pi','4/3\pi','5/3\pi','2\pi'})
yticks([0:pi/3:2*pi])
yticklabels({'0','1/3\pi','2/3\pi','\pi','4/3\pi','5/3\pi','2\pi'})
xlim([-0.3 2*pi+0.4])
ylim([-0.3 2*pi+0.3])
set(gca, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin')
title('(a) U_1')
axis equal
subplot(2,2,2)
for i = 1:4
    vertices = vertices2{i};
    patch(vertices(:,1),vertices(:,2),'blue','FaceAlpha',.1)
    hold on
end
xticks([0:pi/3:2*pi])
xticklabels({'0','1/3\pi','2/3\pi','\pi','4/3\pi','5/3\pi','2\pi'})
yticks([0:pi/3:2*pi])
yticklabels({'0','1/3\pi','2/3\pi','\pi','4/3\pi','5/3\pi','2\pi'})
xlim([-0.3 2*pi+0.4])
ylim([-0.3 2*pi+0.3])
set(gca, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin')
title('(a) U_2')
axis equal
subplot(2,2,3)
for i = 1:4
    vertices = vertices3{i};
    patch(vertices(:,1),vertices(:,2),'red','FaceAlpha',.1)
    hold on
end
xticks([0:pi/3:2*pi])
xticklabels({'0','1/3\pi','2/3\pi','\pi','4/3\pi','5/3\pi','2\pi'})
yticks([0:pi/3:2*pi])
yticklabels({'0','1/3\pi','2/3\pi','\pi','4/3\pi','5/3\pi','2\pi'})
xlim([-0.3 2*pi+0.4])
ylim([-0.3 2*pi+0.3])
set(gca, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin')
title('(a) U_3')
axis equal
subplot(2,2,4)
%plot(vertices4(:,1),vertices4(:,2),'rx','lineWidth',2)
hold on
patch(vertices1(:,1),vertices1(:,2),'green','FaceAlpha',.1)
hold on
for i = 1:4
    vertices = vertices2{i};
    patch(vertices(:,1),vertices(:,2),'blue','FaceAlpha',.1)
    hold on
end
hold on
for i = 1:4
    vertices = vertices3{i};
    patch(vertices(:,1),vertices(:,2),'blue','FaceAlpha',.1)
    hold on
end
xticks([0:pi/3:2*pi])
xticklabels({'0','1/3\pi','2/3\pi','\pi','4/3\pi','5/3\pi','2\pi'})
yticks([0:pi/3:2*pi])
yticklabels({'0','1/3\pi','2/3\pi','\pi','4/3\pi','5/3\pi','2\pi'})
xlim([-0.3 2*pi+0.4])
ylim([-0.3 2*pi+0.3])
set(gca, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin')
title('(d) flat torus')
axis equal
function vertices = transform(U1_x,U1_y)
vertices = [U1_x(1) U1_y(1); U1_x(2) U1_y(1); U1_x(2) U1_y(2); U1_x(1) U1_y(2); U1_x(1) U1_y(1)];
end
