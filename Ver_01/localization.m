function [x, y] = localization(posData)
array = cell(5, 2);
j = 1;
for i = 1:2:10
    [a, b] = circcirc(posData(i,1), posData(i,2), posData(i,4), posData(i+1,1), posData(i+1,2), posData(i+1,4));
    array{j} = a;
    array{j,2} = b;
    j = j+1;
end

tempX = 0;
tempY = 0;

% X value check
if round(array{1}(1)) == round(array{2}(1)) || round(array{1}(1)) == round(array{2}(2))
    tempX = array{1}(1);
else
    tempX = array{1}(2);
end

%Y value check
if round(array{1,2}(1)) == round(array{2,2}(1)) || round(array{1,2}(1)) == round(array{2,2}(2))
    tempY = array{1,2}(1);
else
    tempY = array{1,2}(2);
end

 x = tempX;
 y = tempY;

end

