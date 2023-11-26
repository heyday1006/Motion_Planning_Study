%function [xSample]=sphereworld_sample(world,...)
%Generate samples using the specified distribution, until sphereworld_isCollision
%returns  false for a sample, or a maximum number of trials  nbTrials
%is reached.
% Optional arguments
%   'distribution',distribution : Selects the type of distribution to be used. Possible values for distribution are ?uniform? and ?Gaussian?.
%   'size',sz : Size parameter for the distribution (default value: sz=5). For the ?uniform? distribution, the resulting domain of the random variable is the square [?maxSize, maxSize]2. For the ?Gaussian? distribution, sz represents the variance.
%   'mean',mu (dim. [2 × 1]): Mean of the distribution (default value: mu=[0;0]).
function [xSample]=sphereworld_sample(world,varargin)
sz=10;
distribution='uniform';
mu=[0;0];
nbTrials=1000;

%optional parameters
ivarargin=1;
while ivarargin<=length(varargin)
    switch lower(varargin{ivarargin})
        case 'size'
            ivarargin=ivarargin+1;
            sz=varargin{ivarargin};
        case 'distribution'
            ivarargin=ivarargin+1;
            distribution=varargin{ivarargin};
        case 'mean'
            ivarargin=ivarargin+1;
            mu=varargin{ivarargin};
       otherwise
            error(['Argument ' varargin{ivarargin} ' not valid!'])
    end
    ivarargin=ivarargin+1;
end

distribution=lower(distribution);

for iTrial=1:nbTrials
    switch distribution
        case 'uniform'
            xSample=2*sz*(rand(2,1)-0.5)+mu;
        case 'gaussian'
            xSample=sz*randn(2,1)+mu;
        otherwise
            error('Distribution type not recognized')
    end
    if ~sphereworld_isCollision(world,xSample)
        break %no collision with a sphere
    end
end