% Coded by Hyeokjun Kwon
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
X_max = 500;
Y_max = 500;
trans_posX = randi([-X_max,X_max],1,1);
trans_posY = randi([-Y_max,Y_max],1,1);
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
fc = 700;                           % frequency (MHz)
transPower = 50;                    % Transmission Power (dB)
antGain = 3;                        % Antenna Gain (dB)
Bc = 20e6;                                  % Bandwidth (Hz)
noisePower = -174+10*log10(Bc) - 30;  % Thermal noise (dB): -174(dBm) + 10log10(bandwidth) -30

%% Instruction 2 - Drone move and save locations
% Drone location info matrix
posData = [0,0,0];
dronePos = [0 0 30];        % Initial location: x = 0, y = 0, altitude = 30
posData = dronePos;         % The first location

% Move the drone randomly
%    a. flying 50m in random direction
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
% posData = horzcat(posData, d);      % posData = [x, y, altitude, distance]

% path loss of each location of the drone
fsplMatrix = zeros(10,1);
for z = 1:10
    pl = pathLoss(posData, transPos, fc, c, d, z); % free Space Path Loss 
    fsplMatrix(z,1) = pl;
end
posData = horzcat(posData, fsplMatrix);      % posData = [x, y, altitude, pathloss]

% Recieved signal of each location
strMatrix = zeros(10,1);
for k = 1:10
    recStr = recSig(posData, transPower, antGain, noisePower, k);
    strMatrix(k,1) = recStr;
end
posData = horzcat(posData, strMatrix);

%{
% Received strength & path loss at the reference distance(d=1m)
plx0 = 20*log10(fc) - 27.55;
plx0L = sqrt(10^(plx0/10));
  transPowerL = sqrt(10^(transPower/10));
  antGainL = sqrt(10^(antGain/10));
  sigStrength = transPowerL * antGainL;
  noise = sqrt(10^(noisePower/10)) * randn(1);  % dB to linear
Prx0 = sigStrength / plx0L + noise;
%}

% Estimate distances from each drone to the transmitter
% cat the distances to posData
estimatedD = estimatedDist(posData,fc);
posData = horzcat(posData, estimatedD);


%% Instruction 4 - Calculate the location of the source
% draw a circle for each location
for h = 1:10
    drawCircle(posData(h,1), posData(h,2), posData(h,3), posData(h,6));
end

% find a overlaped location
[posX, posY] = localization(posData);
posX = round(posX);
posY = round(posY);
fprintf("Estimated locatioin of the transmitter is X: %f, Y: %f\n", posX, posY);