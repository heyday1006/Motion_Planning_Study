%Returns a sample x of a random variable X that can assume values in xDistr
%with probability given by fDistr (that is, p(x = xDistr(i)) = fDistr(i)). 
%Input arguments
%   xDistr (dim. [NValues × 1]): set of values that X can assume.
%   fDistr (dim. [NValues × 1]): array describing the probability of each
%       value in xDistr. Note: the function normalizes fDistr so that it sums to one. 
%Output arguments
%   x (dim. [1 × 1]): a sample from the distribution. Internally, the
%       function uses rand() and the cumulative distribution of X to generate x. 
function x=rand_discrete(xDistr,fDistr)
%normalize to one
fDistr=fDistr/sum(fDistr);

%cumulative distribution
cumfDistr=cumsum(fDistr);
cumfDistr(end)=1;

%sample from uniform, and then invert cumulative distribution
s=rand();
x=xDistr(find(s<cumfDistr,1,'first'));