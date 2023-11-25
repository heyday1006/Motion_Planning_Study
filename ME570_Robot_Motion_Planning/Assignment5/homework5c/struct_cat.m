%Concatenate two structures with possibly different fields
%function s1=struct_cat(dim,s1,s2)
%Inputs
%   s1,s2   two struct arrays
%   dim     dimension for concatenation (same as the standard CAT command)
%Outputs
%   s1      Same as the input s1 but with s2 concatenated to it
%Note: this function is limited to 1-D and 2-D arrays
function s1=struct_cat(dim,s1,s2)
fNames=fieldnames(s2);
sz1=size(s1);
sz2=size(s2);
idxCompatibility=[1:dim-1 dim+1:2];
flagCompatibility=all(sz1(idxCompatibility)==sz2(idxCompatibility));
if ~flagCompatibility
    error('s1,s2 must be of compatible sizes')
end

for iRow=1:sz2(1)
    for iColumn=1:sz2(2)
        idx2={iRow, iColumn};
        idx1=idx2;
        idx1{dim}=idx1{dim}+sz1(dim);
        for iField=1:numel(fNames)
            thisField=fNames{iField};
            s1(idx1{:}).(thisField)=s2(idx2{:}).(thisField);
        end
    end
end
