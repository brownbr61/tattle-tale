function dataOut = lowPassFilter(dataIn,nSignificantSamples)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
n = nSignificantSamples;
filter = (1-1/n).^(0:n)/sum((1-1/n).^(0:n));
dataOut = conv(dataIn,filter);
dataOut = dataOut(n:(end-n));
end