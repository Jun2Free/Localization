function [disMat] = distance(transPos, posData)
disMat = zeros(10, 1);

for i = 1:10
    d = sqrt((transPos(1) - posData(i,1))^2 + (transPos(2) - posData(i,2))^2 + (transPos(3) - posData(i,3)));
    disMat(i,1) = d;
end

end

