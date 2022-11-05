%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% cdf for the linear and nonlinear algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

load('lintraj_bound.mat')
de_bound = de;

load('lintraj_paral.mat')
de_parall = de;

load('lintraj_square.mat')
de_squre = de;

a = cdfplot(de_bound); 
set(a, 'LineStyle', '-', 'Color', 'r','LineWidth',1);
hold on; grid on;
b = cdfplot(de_parall)
set(b, 'LineStyle', '-', 'Color', 'b','LineWidth',1);
c = cdfplot(de_squre)
set(c, 'LineStyle', '-', 'Color', 'g','LineWidth',1);

xlabel('Error Distance(m)', FontSize=14)
ylabel('CDF', FontSize=14)

legend("Boundary Model", "Parallel Model", "Squre Model", FontSize=14);
