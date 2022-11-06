%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% cdf for the linear and nonlinear algorithm - 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

load('lin_3D.mat')
de_lin = de;

load('nonlin_3D.mat')
de_nonlin = de;

a = cdfplot(de_lin); 
set(a, 'LineStyle', '-', 'Color', 'r','LineWidth',1);
hold on; grid on;
b = cdfplot(de_nonlin)
set(b, 'LineStyle', '-', 'Color', 'b','LineWidth',1);

xlabel('Error Distance(m)', FontSize=14)
ylabel('CDF', FontSize=14)

legend("LLS", "NLLS", FontSize=14);
