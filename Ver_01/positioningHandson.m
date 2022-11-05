clear all; clc; close all;

%% Introduction
% This example shows how to localize a randomly dropped transmitter in the limitted area
% The locationi of the transmitter can be calculated by 10 different locations of Drone 
% Each location of Drone has its signal strength.
% This signal strength is influenced by SNR
% For the first experiment, SNR will be represented as a fixed constant
% In the further experiment, various factors of drone and noise will be
% considered. 

%% Instruction 1 - Build a map & define constants and a transmitter
% 1. Area is 1km * 1km 
% 2. Randomly drop a transmitter on the map
min = -500;
max = 500;
%trans_posX = (max-min).*rand(1) + min;
%trans_posY = (max-min).*rand(1) + min;
trans_posX = randi(500);
trans_posY = randi(500);
trans_height = 10;
transPos = [trans_posX, trans_posY, trans_height];
fprintf('Transmitter location is X: %f, Y: %f, Z: %f\n', trans_posX, trans_posY, trans_height);
figure(2);
hold on
plot3(trans_posX, trans_posY, trans_height, '-o','Color','r','MarkerSize',10,...
    'MarkerFaceColor','#D8FFFF')
title('Locatioin of Transmitter');
xlabel('x-value');
ylabel('y-value');
zlabel('Height');

% list of constants needed for calculation 
c = 299792458.0;                    % Speed of light in m/s
fc = 2.1e9;                         % frequency
transPower = 0;                     % Transmission Power
antGain = 3;                        % Antenna Gain
noisePower = 10;                    % Noise power

%% Instruction 2 - Drone move and save locations
% Drone location info matrix
posData = [0,0,0];
dronePos = [0 0 30];        % Initial location: x = 0, y = 0, altitude = 30
posData = dronePos;         % The first location info

% Move the drone randomly
%    a. flying 18m in random direction
%    b. do same operation and calculation 10 times
posNum = 1;
while true
    isHave = 0;
    dronePos = randPos(dronePos);
    for j = 1:size(posData,1)
        if dronePos == posData(j,:)
            isHave = 1;
            break
        end
    end
    if isHave ~= 1
        if posNum < 10
            posData = vertcat(posData,dronePos);
            posNum = posNum + 1;
        elseif posNum == 10
            break
        end
    end
end

%% Movement of the drone - plot3
figure(1);
plot3(posData(:,1), posData(:,2), posData(:,3), '-o','Color','b','MarkerSize',10,...
    'MarkerFaceColor','#D9FFFF')
title('Locatioin of Drone');
xlabel('x-value');
ylabel('y-value');
zlabel('Height');

%% Instructioin 3 - Calculate data 
% distance btw the BS and the drone
d = distance(transPos, posData);    
posData = horzcat(posData, d);      % posData = [x, y, altitude, distance]

% path loss of each location of the drone
pl = real(pathLoss(posData, transPos, fc, c)); % free Space Path Loss 
posData = horzcat(posData, pl);      % posData = [x, y, altitude, distance, pathloss]

% Recieved signal strength of each location
 % RSSI = Transmit Power(상수) + antenna gain(상수) - path loss + noise
receivedSt = recSig(posData, transPower, antGain, noisePower);
posData = horzcat(posData, receivedSt);

%% Instruction 4 - Calculate the location of the source
% draw a circle for each location
for h = 1:10
    drawCircle(posData(h,1), posData(h,2), posData(h,3), posData(h,4));
end
% find a overlaped location
[posX, posY] = localization(posData);
posX = round(posX);
posY = round(posY);
fprintf("Estimated locatioin of the transmitter is X: %f, Y: %f\n", posX, posY);

