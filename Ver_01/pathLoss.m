function [fsplMatrix] = pathLoss(posData, transPos, fc, c)

fsplMatrix = zeros(10,1);
% Effective transmitter antenna height
effTr = transPos(3) - 1.0;
% Effective drone antenna height
effDr = zeros(10,1);
for i = 1:10
    effDr(i,1) = posData(3) - 1.0;
end

% Break point distance
dBP = zeros(10,1);
for j = 1:10
    dBP(j,1) = 4 * effTr * effDr(j,1) * fc / c;
end

% Free Space Path Loss
for z = 1:10
    if posData(z,4) < dBP(z,1)
        fspl = 20.0 * log10(posData(z,4)) + 28.0 + 20.0 * log10(fc/1e9);
    else
        fspl = 40.0 * log10(posData(z,4)) + 7.8 - 18.0 * log10(effTr) - ...
            18.0 * log10(effDr(z,1)) + 2.0 * log10(fc/1e9);
    end
    fsplMatrix(z,1) = fspl;
end


