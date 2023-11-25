%function [vertexEffectorTransf,vertices1Transf,vertices2Transf]=twolink_kinematicMap(theta)
%The function returns the coordinate of the end effector, plus the vertices of
%the links, all transformed according to $ _1, _2$.
%INPUT:  theta:  [2x1]vec two joint angles for the two-link manipulator
%OUTPUT: vertexEffectorTransf: [2x1]vec coordinates of end effector in world frame
%        vertices1Trans:       [2x4]vec transformed vertices in twolink_polygon()
%        vertices2Trans:       [2x12]vectransformed vertices in twolink_polygon()
function [vertexEffectorTransf,vertices1Transf,vertices2Transf]=twolink_kinematicMap(theta)
%Use the results from Question  q:kinematicMapDerivation to guide your
%implementation. This function must use twolink_polygons to obtain the vertices
%of the polygons of the matrix, and it must use rot2d from Question  q:rot2d.
%Note that here we are simply computing the vertices of the transformed polygons,
%without plotting them. The next function will be used to plot the transformed
%vertices.

%coordinates of end effector in B2 frame
p_eff_B2 = [5;0];
%coordinates of vertices of two polygons of two-link manipulator
[vertices1,vertices2]=twolink_polygons();
%rotation matrix from B1 to W with angle theta(1)
Rot_B1W = rot2d(theta(1));
%translation matrix from B1 to W
Trans_B1W = [0;0];
%rotation matrix from B2 to B1 with angle theta(2)
Rot_B2B1 = rot2d(theta(2));
%translation matrix from B2 to B1
Trans_B2B1 = [5;0];
vertexEffectorTransf = transform_twoJointAngles(p_eff_B2,Rot_B1W,Trans_B1W,Rot_B2B1,Trans_B2B1);
vertices1Transf = transform_oneJointAngles(vertices1,Rot_B1W,Trans_B1W);
vertices2Transf = transform_twoJointAngles(vertices2,Rot_B1W,Trans_B1W,Rot_B2B1,Trans_B2B1);

function p_W = transform_oneJointAngles(p_B1,Rot_B1W,Trans_B1W)
    p_W = Rot_B1W*p_B1 + Trans_B1W;
    
function p_W = transform_twoJointAngles(p_B2,Rot_B1W,Trans_B1W,Rot_B2B1,Trans_B2B1)
    p_W = Rot_B1W*Rot_B2B1*p_B2 + Rot_B1W*Trans_B2B1 + Trans_B1W;


