clear all
close all
%% Parameter Setup
N_bs = 10;                         % # Base station
X_max = 300;                        % maximum x-domain offset from the origin [m]
Y_max = 300;                        % maximum y-domain offset from the origin [m]

c = 299792458.0;                    % Speed of light in m/s
fc = 700;                           % frequency (MHz)
Bc = 20e6;                          % Bandwidth (Hz)
P_Tx_dB = 50;                       % Transmission Power (dB)
G_t_dB = 3;                         % Antenna Gain (dB)

N_0_dB = -174+10*log10(Bc) - 30;    % Thermal noise (dB): -174(dBm) + 10log10(bandwidth) -30

N_iter = 5000;                      % # iteration

% for ii= 1:N_iter
%% Networks Setup
% generate locations of BS
l_x = randi([-X_max,X_max],N_bs,1);
l_y = randi([-X_max,X_max],N_bs,1);

l_BS = [l_x l_y];

% generate locations of target user
l_x = randi([-X_max,X_max],1,1);
l_y = randi([-X_max,X_max],1,1);

l_U = [l_x l_y];

%% Recevied Signal
dist = sqrt(sum(abs(l_BS - l_U).^2,2)) ;
PL_dB = 20*log10(fc) + 20*log10(dist) + 32.44 ; % free-space path loss

r_sig_pw_dB = G_t_dB + P_Tx_dB - PL_dB ; % received signal power in dB
r_sig_lin = sqrt(10.^(r_sig_pw_dB/10)) ; % received signal in linear

N_0 = sqrt(10^(N_0_dB/10))*randn(N_bs,1) ; % white Gaussian noise

r_sig_tot = r_sig_lin + N_0 ; % receivd signal including noise

SNR = r_sig_pw_dB - N_0_dB ; % received signal-to-noise-radio

%% Distance Estimation
r_sig_tot_pw_dB = 10*log10(abs(r_sig_tot).^2) ; % received signal power in dB

dist_est = 10.^(-( r_sig_tot_pw_dB - G_t_dB - P_Tx_dB + 32.44 + 20*log10(fc) )/20) ; % estimate distance by inverse expressions

%% Localization
l_BS = horzcat(l_BS, dist_est)
%fit = NonlinearModelFit[l_BS, Norm[{x, y} - {x0, y0}], {x0,y0}, {x,y}, Weights -> 1/observations^2]
%end


