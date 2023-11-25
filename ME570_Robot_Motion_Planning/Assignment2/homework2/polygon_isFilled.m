%function [flag]=polygon_isFilled(vertices)
%Checks the ordering of the vertices, and returns whether the polygon is filled
%in or not.
function [flag]=polygon_isFilled(vertices)
[r_num,c_num]=size(vertices);%obtain the dimension information of the given vertices set

VecSet=zeros(r_num,c_num);%memory allocation to store vectors between two consecutive vertices.
SubVec=zeros(r_num,c_num);%memory allocation to store subsequent vectors.

%Assignment for vectors.
VecSet(:,1:c_num-1)=vertices(:,2:c_num);
VecSet(:,c_num)=vertices(:,1);
VecSet=VecSet-vertices;

SubVec(:,1:c_num-1)=VecSet(:,2:c_num);
SubVec(:,c_num)=VecSet(:,1);

angleSum=0;%variable that records the sum of angles line segments rotated.

%compute the sum of angle that line segments rotate.
for index=1:c_num
    %compute the rotation angle between two consecutive vectors, and
    %perform angle summation.
    angleSum=angleSum+edge_angle([0;0],VecSet(:,index),SubVec(:,index),'signed');
end

%judge the sign of the angle summation.
if angleSum>0
    %positive angle summation indicates that the vertices are placed in
    %counterclockwise order.
    flag=true;
    return;
elseif angleSum<0
    %negative angle summation indicates that the vertices are placed in
    %clockwise order.
    flag=false;
    return;
else
    flag=NaN;
    return;
end




