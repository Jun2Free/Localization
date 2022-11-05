%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% For the nonlinear least square algorithm, nonlinear function sumup
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all;
X_max = 500;                        % maximum x-domain offset from the origin [m]
Y_max = 500;                        % maximum y-domain offset from the origin [m]

c = 299792458.0;                    % Speed of light in m/s
fc = 2400;                           % frequency (MHz)
Bc = 20e6;                          % Bandwidth (Hz)
P_Tx_dB = 33;                       % Transmission Power (dB)
G_t_dB = 0;                         % Antenna Gain (dBi)
N_0_dB = -174+10*log10(Bc) - 30;

de = [0];                 % distance error  vector
dr_dist = [0]       % Total distance of Drone
miss_cnt = 0;       % Total number of failed localization
hit_cnt = 0;        % Total number of success localization

tot_num_localization = 1;
while tot_num_localization <= 100

    est_T = [0 0 1000];        % estimated locatioin vector
    minima = 1000;

    % Target Location
    l_xt = randi([0, X_max],1,1);
    l_yt = randi([0, Y_max],1,1);
    l_TG = [l_xt l_yt]

    % Drone Location
    l_x = 0;
    l_y = 0;
    l_DR = [l_x l_y];

    dr_move = 1;

    while dr_move < 700
        %{
        % Trajectory #2.Boundary
        if (dr_move >= 300) && (dr_move < 400)
            l_x = l_x + 0;
            l_y = l_y - 5;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 200) && (dr_move < 300)
            l_x = l_x - 5;
            l_y = l_y + 0;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 100) && (dr_move < 200)
            l_x = l_x - 0;
            l_y = l_y + 5;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move > 0) && (dr_move < 100)
            l_x = l_x + 5;
            l_y = l_y + 0;
            l_DR = [l_DR; l_x l_y];
        end
        %}
        % Trajectory #1. Parallel 
        
        if (dr_move >= 600) && (dr_move < 700)
            l_x = l_x - 5;
            l_y = l_y + 0;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 580) && (dr_move < 600)
            l_x = l_x + 0;
            l_y = l_y + 5;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 480) && (dr_move < 580)
            l_x = l_x + 5;
            l_y = l_y + 0;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 460) && (dr_move < 480)
            l_x = l_x + 0;
            l_y = l_y + 5;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 360) && (dr_move < 460)
            l_x = l_x - 5;
            l_y = l_y + 0;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 340) && (dr_move < 360)
            l_x = l_x + 0;
            l_y = l_y + 5;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 240) && (dr_move < 340)
            l_x = l_x + 5;
            l_y = l_y + 0;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 220) && (dr_move < 240)
            l_x = l_x + 0;
            l_y = l_y + 5;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 120) && (dr_move < 220)
            l_x = l_x - 5;
            l_y = l_y + 0;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move >= 100) && (dr_move < 120)
            l_x = l_x - 0;
            l_y = l_y + 5;
            l_DR = [l_DR; l_x l_y];
        elseif (dr_move > 0) && (dr_move < 100)
            l_x = l_x + 5;
            l_y = l_y + 0;
            l_DR = [l_DR; l_x l_y];
        end
        dr_move = dr_move+1;

        %% Measured Recevied Signal
        % dist = sqrt(sum(abs([l_DR(dr_move,1) l_DR(dr_move,2)] - l_TG).^2,2)) ;       % estimated distance
        dist = sqrt(sum(abs([l_DR(:,1) l_DR(:,2)] - l_TG).^2,2)) ;       % estimated distance
        PL_dB = 20*log10(fc) + 20*log10(dist) - 27.55; % estimated path loss

        r_sig_dB = G_t_dB + P_Tx_dB - PL_dB; % received signal power in dB
        r_sig_lin = sqrt(10.^(r_sig_dB/10)) ;

        % measured value
        N_0 = 10^(N_0_dB/10)*randn(1,1);
        r_sig_tot = r_sig_lin + N_0 ;
        r_sig_tot_pw_dB = 10*log10(abs(r_sig_tot).^2) ;

        %% Distance Estimation
        dist_meas = 10.^((P_Tx_dB + G_t_dB - r_sig_tot_pw_dB + 27.55 - 20*log10(fc))/20) ; % measured distance by inverse expressions
        meas_PL_dB = 20*log10(fc) + 20*log10(dist_meas) - 27.55; % measured pathloss

        %% Localization; RSSI based
        x0 = [l_x+0.01; l_y+0.01];
        % nonlinear
        fun = @(x)sum((20*log10(dist_meas) - 20*log10(sqrt(sum(abs([l_DR(:,1) l_DR(:,2)] - [x(1) x(2)]).^2,2)))).^2)
        %fun = @(x) (PL_dB - 20*log10(fc) - 20*log10(10.^((P_Tx_dB + G_t_dB - (10*log10(abs(((sqrt(10.^((G_t_dB + P_Tx_dB - (20*log10(fc) + 20*log10((sqrt(sum(abs([l_DR(dr_move,1) l_DR(dr_move,2)] - [x(1) x(2)]).^2,2)))) - 27.55))/10))) + N_0)).^2)) + 27.55 - 20*log10(fc))/20)) + 27.55).^2;
        %fun = @(x)(20*log10(dist) - 20*log10(sqrt(sum(abs([l_DR(dr_move,1) l_DR(dr_move,2)] - [x(1) x(2)]).^2,2)))).^2
        [x, fval] = fminunc(fun, x0)

        %est_T = [est_T; x(1) x(2) fval];

        % find the global minima
        if(fval <= minima)
            minima = fval;
            est_T = [est_T; x(1) x(2) fval];
        else
            est_T = [est_T; est_T(dr_move-1,:)];
        end

        %err_dist = sqrt((l_TG(1)-est_T(dr_move,1)).^2 + (l_TG(2)-est_T(dr_move,2)).^2);
        err_dist = sqrt(sum(est_T(size(est_T,1),1:2) - l_TG).^2);
        move_dist_m = dr_move*5;    % drone moving distance
        
    end
    de = [de; err_dist];        % for mean value of error distance
    dr_dist = [dr_dist; move_dist_m];
    tot_num_localization = tot_num_localization + 1;

    if err_dist <= 5*sqrt(2)
        newline
        fprintf('Target is localized');
        hit_cnt = hit_cnt + 1;
    else
        fprintf('Localization is failed');
        miss_cnt = miss_cnt + 1;
    end
end

%% Performance data
% Mean of Total distance of Drone
c = newline
fprintf("Mean of Total distance of Drone");
mean(dr_dist(2:size(dr_dist)))

% Mean of Error Distance
fprintf("Mean of Error distance");
mean(de(2:size(de)))

%% Plot
xlabel('X Position', FontSize = 14);
ylabel('Y Position', FontSize = 14);
plot(l_DR(:,1), l_DR(:,2),'k-','LineWidth',1); hold on;  % Drone Trajectory
plot(est_T(2:size(est_T),1), est_T(2:size(est_T),2), 'b:o','LineWidth',1); hold on;   % Estimated Location of the target
plot(l_TG(1), l_TG(2), 'r*');   % Target Location
legend('UAV Trajectory', 'Estimated Target Location', 'Target Location', FontSize = 14);