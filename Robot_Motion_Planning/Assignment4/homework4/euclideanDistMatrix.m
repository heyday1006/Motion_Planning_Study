%function dSq=euclideanDistMatrix(A,B)
%Compute the Euclidean squared distance between each sample of A and each sample
%of B
function dSq=euclideanDistMatrix(A,B)

if ~exist('B','var') || isempty(B)
    B=A;
end

NA=size(A,2);
NB=size(B,2);

dSq=sum(A.^2,1)'*ones(1,NB)+ones(NA,1)*sum(B.^2,1)-2*A'*B;
