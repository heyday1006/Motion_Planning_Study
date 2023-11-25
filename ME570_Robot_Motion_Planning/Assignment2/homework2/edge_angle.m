%function [edgeAngle]=edge_angle(vertex0,vertex1,vertex2,type)
%Compute the angle between two edges  vertex0-- vertex1 and  vertex0-- vertex1
%having an endpoint in common. The angle is computed by starting from the edge 
%vertex0-- vertex1, and then ``walking'' in a counterclockwise manner until the
%edge  vertex0-- vertex1 is found.
%The function returns NaN if vertex1 or vertex2 coincides with vertex0
function [edgeAngle]=edge_angle(vertex0,vertex1,vertex2,angleType)
vec1=(vertex1-vertex0)/norm(vertex1-vertex0);%compute the normalized vector from vertex0 to vertex1.
vec2=(vertex2-vertex0)/norm(vertex2-vertex0);%compute the normalized vector from vertex0 to vertex2.

cosine_theta=vec1'*vec2;%compute the cosine value of the angle using the inner product of two vectors.
sine_theta=[0 0 1]*cross([vec1;0],[vec2;0]);%compute the sine value of the angle.

edgeAngle=atan2(sine_theta,cosine_theta);

%compute the edge angle
if strcmpi(angleType,'unsigned')
    edgeAngle=mod(edgeAngle+2*pi,2*pi);
elseif strcmpi(angleType,'signed')
    return;
else
    disp('input error');
    edgeAngle=NaN;
    return;
end



        
        