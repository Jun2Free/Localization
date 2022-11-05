function [distanceMat] = estimatedDist(posData, fc)
distanceMat = zeros(10,1);

for i =1:10
    fspl = 20*log10(posData(i,4));           % transfer to dB
    d = 10^((fspl-20*log10(fc)+27.55)/20);
    distanceMat(i,1) = d;
end


end