%function [edgeAngle]=edge_angle(vertex0,vertex1,vertex2,type)
%Compute the angle between two edges  vertex0-- vertex1 and  vertex0-- vertex1
%having an endpoint in common. The angle is computed by starting from the edge 
%vertex0-- vertex1, and then ``walking'' in a counterclockwise manner until the
%edge  vertex0-- vertex1 is found.
%The function returns NaN if vertex1 or vertex2 coincides with vertex0
function [edgeAngle]=edge_angle(vertex0,vertex1,vertex2,angleType)
%tolerance to check for coincident points
tol=2*eps;
if ~exist('angleType','var') || isempty(angleType)
    angleType='signed';
end

%compute vectors corresponding to the two edges, and normalize
vec1=vertex1-vertex0;
vec2=vertex2-vertex0;

normVec1=norm(vec1);
normVec2=norm(vec2);
if normVec1<tol || normVec2<tol
    %vertex1 or vertex2 coincides with vertex0, abort
    edgeAngle=NaN;
    return
end

vec1=vec1/normVec1;
vec2=vec2/normVec2;

cAngle=vec1'*vec2;
sAngle=[0;0;1]'*cross([vec1;0],[vec2;0]);

edgeAngle=atan2(sAngle,cAngle);

switch lower(angleType)
    case 'signed'
        %nothing to do
    case 'unsigned'
        edgeAngle=mod(edgeAngle+2*pi,2*pi);
    otherwise
        error('Angle type not recognized')
end

%edge_angle([0;0],[-1;-1],[0;1])
        