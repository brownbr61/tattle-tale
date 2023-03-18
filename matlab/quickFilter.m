function dataOut = quickFilter(dataIn,bitCt)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
divisor = 2^bitCt;
dataOut = zeros([1, ceil(length(dataIn)/divisor)]);
for i = 1:length(dataIn)
  dataOut(ceil(i/divisor)) = dataOut(ceil(i/divisor)) + dataIn(i);
  if (mod(i,divisor) == divisor - 1)
    dataOut(ceil(i/divisor)) = (ceil(i/divisor))/divisor;
  end
end
dataOut(end) = [];