function [nextPos] = randPos(prevPos)
% this model is for drone that flyig to random direction
number = randi(4);
nextPos = prevPos;

% move 18m to random direction
if number == 1   % right
    nextPos(1) = nextPos(1) + 18;
    if nextPos(1) > 500 
        nextPos(1) = 500;
    end  
elseif number == 2   % left
    nextPos(1) = nextPos(1) - 18;
    if nextPos(1) < -500 
        nextPos(1) = -500;
    end
elseif number == 3   % up
    nextPos(2) = nextPos(2) + 18;
    if nextPos(2) > 500 
        nextPos(2) = 500;
    end
elseif number == 4   % down
    nextPos(2) = nextPos(2) - 18;
    if nextPos(2) < -500 
        nextPos(2) = -500;
    end
end

end

