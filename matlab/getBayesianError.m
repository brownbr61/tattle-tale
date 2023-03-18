function bayesianError = getBayesianError(filteredData,min,max)
%PLOTBAYESIANERROR Summary of this function goes here
%   Detailed explanation goes here
bayesianError = sum(filteredData < min | filteredData > max)/length(filteredData);
end

