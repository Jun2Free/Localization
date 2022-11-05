function [fsplMatrix] = pathLoss(posData, transPos, fc, c, d, z)

% Free Space Path Loss
    fspl = 20.0 * log10(d(z)) + 20.0 * log10(fc) - 27.55; % frequency is MHz
    fspl = sqrt(10^(fspl/10));      % trans to linear
    fsplMatrix = fspl;
end
