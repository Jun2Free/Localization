clear all; close all;
%% Parameter Setup
X_max = 500;                        % maximum x-domain offset from the origin [m]
Y_max = 500;                        % maximum y-domain offset from the origin [m]
Z_max = 50;

c = 299792458.0;                    % Speed of light in m/s
fc = 2400;                           % frequency (MHz)
Bc = 20e6;                          % Bandwidth (Hz)
P_Tx_dB = 33;                       % Transmission Power (dB)
G_t_dB = 3;                         % Antenna Gain (dBi)
N_0_dB = -174+10*log10(Bc) - 30;    % Thermal noise (dB): -174(dBm) + 10log10(bandwidth) -30

de = [0];                 % distance error  vector
dr_dist = [0]       % Total distance of Drone
miss_cnt = 0;       % Total number of failed localization
hit_cnt = 0;        % Total number of success localization

tot_num_localization = 1;
while tot_num_localization <= 500

    est_T = [0 0 0];             % estimated locatioin vector

    % Target Location
    l_xt = randi([0, X_max],1,1);
    l_yt = randi([0, X_max],1,1);
    l_zt = randi([0, Z_max],1,1);
    l_TG = [l_xt l_yt l_zt]

    % Drone Location
    l_x = 0;
    l_y = 0;
    l_z = 30;
    l_DR = [l_x l_y l_z];

    dr_move = 1;

    while dr_move < 700         % for boundary trajectory
        rt_cnt = 1;     % movement count in a loop
        if size(l_DR,1) == 1        % for the first loop, get two locations of drone
            while rt_cnt < 3
                % Trajectory #2.Boundary
                %{
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
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 580) && (dr_move < 600)
                    l_x = l_x + 0;
                    l_y = l_y + 5;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 480) && (dr_move < 580)
                    l_x = l_x + 5;
                    l_y = l_y + 0;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 460) && (dr_move < 480)
                    l_x = l_x + 0;
                    l_y = l_y + 5;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 360) && (dr_move < 460)
                    l_x = l_x - 5;
                    l_y = l_y + 0;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 340) && (dr_move < 360)
                    l_x = l_x + 0;
                    l_y = l_y + 5;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 240) && (dr_move < 340)
                    l_x = l_x + 5;
                    l_y = l_y + 0;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 220) && (dr_move < 240)
                    l_x = l_x + 0;
                    l_y = l_y + 5;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 120) && (dr_move < 220)
                    l_x = l_x - 5;
                    l_y = l_y + 0;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 100) && (dr_move < 120)
                    l_x = l_x - 0;
                    l_y = l_y + 5;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move > 0) && (dr_move < 100)
                    l_x = l_x + 5;
                    l_y = l_y + 0;
                    l_DR = [l_DR; l_x l_y l_z];
                end
                
                rt_cnt = rt_cnt+1;
                dr_move = dr_move+1;
            end
        end
        if size(l_DR,1) >= 3 % from the second loop, get two locations of drone for each loop
            rt_cnt = 1;
            while rt_cnt <= 3
                % Trajectory #2.Boundary
                %{
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
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 580) && (dr_move < 600)
                    l_x = l_x + 0;
                    l_y = l_y + 5;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 480) && (dr_move < 580)
                    l_x = l_x + 5;
                    l_y = l_y + 0;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 460) && (dr_move < 480)
                    l_x = l_x + 0;
                    l_y = l_y + 5;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 360) && (dr_move < 460)
                    l_x = l_x - 5;
                    l_y = l_y + 0;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 340) && (dr_move < 360)
                    l_x = l_x + 0;
                    l_y = l_y + 5;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 240) && (dr_move < 340)
                    l_x = l_x + 5;
                    l_y = l_y + 0;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 220) && (dr_move < 240)
                    l_x = l_x + 0;
                    l_y = l_y + 5;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 120) && (dr_move < 220)
                    l_x = l_x - 5;
                    l_y = l_y + 0;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move >= 100) && (dr_move < 120)
                    l_x = l_x - 0;
                    l_y = l_y + 5;
                    l_DR = [l_DR; l_x l_y l_z];
                elseif (dr_move > 0) && (dr_move < 100)
                    l_x = l_x + 5;
                    l_y = l_y + 0;
                    l_DR = [l_DR; l_x l_y l_z];
                end
             
                rt_cnt = rt_cnt+1;
                dr_move = dr_move+1;
            end
        end

        %% Recevied Signal
        dist = sqrt(sum(abs(l_DR - l_TG).^2,2)) ;
        PL_dB = 20*log10(fc) + 20*log10(dist) - 27.55 ; % free-space (measured)path loss
        
        r_sig_pw_dB = G_t_dB + P_Tx_dB - PL_dB ; % received signal power in dB
        r_sig_lin = sqrt(10.^(r_sig_pw_dB/10)) ; % received signal in linear
        
        N_0 = sqrt(10^(N_0_dB/10))*randn(size(r_sig_lin,1),1) ; % white Gaussian noise
        r_sig_tot = r_sig_lin + N_0 ; % receivd signal including noise
        r_sig_tot_dB = 10*log10(r_sig_tot);
        r_sig_tot_pw_dB = 10*log10(abs(r_sig_tot).^2) ; % received signal power in dB
       
        %% Distance Estimation
        dist_meas = 10.^(-( r_sig_tot_pw_dB - G_t_dB - P_Tx_dB - 27.55 + 20*log10(fc) )/20) ; % estimate distance
        meas_PL_dB = 20*log10(fc) + 20*log10(dist_meas) - 27.55;

        %% Localization; RSSI based

        % Algorithm 01: Linear Least Square
        % (algorithm by: Hyeon Jeong Jo and Seungku Kim, Indoor Smartphone Localization Based on LOS and NLOS Identificatio, Sensors, Nov. 2018.)
        [~,idx_DR] = sort(dist_meas); % sort disance shorstest to longest
        idx_DR = idx_DR(1:3) ; % Only choose the closest 3 BS

        d_1 = dist_meas(idx_DR(1)) ; % distance
        d_2 = dist_meas(idx_DR(2)) ;
        d_3 = dist_meas(idx_DR(3)) ;

        x_1 = l_DR(idx_DR(1),1) ; % x-domain location
        x_2 = l_DR(idx_DR(2),1) ;
        x_3 = l_DR(idx_DR(3),1) ;

        y_1 = l_DR(idx_DR(1),2) ; % y-domain location
        y_2 = l_DR(idx_DR(2),2) ;
        y_3 = l_DR(idx_DR(3),2) ;

        z_1 = l_DR(idx_DR(1),3) ; % z-domain location
        z_2 = l_DR(idx_DR(2),3) ;
        z_3 = l_DR(idx_DR(3),3) ;

        A = [2*(x_1 - x_2) 2*(y_1 - y_2) 2*(z_1 - z_2); 2*(x_2 - x_3) 2*(y_2 - y_3) 2*(z_2 - z_3)];
        B = [x_1^2 - x_2^2 + y_1^2 - y_2^2 + z_1^2 - z_2^2 - d_1^2 + d_2^2 ; x_2^2 - x_3^2 + y_2^2 - y_3^2 + z_2^2 - z_3^2 - d_2^2 + d_3^2];

        X = pinv(A)*B;    % estimated location

        est_T = [est_T; X(1) X(2) X(3)];
        err_dist = sqrt(sum(abs(X - l_TG').^2)); % distance error [m]
        move_dist_m = dr_move*5;    % drone moving distance
%{
        if err_dist <= 5
            newline
            fprintf('Target is localized');
            hit_cnt = hit_cnt + 1;
            break;
        end
        if dr_move >= 700
            fprintf('Localization is failed');
            miss_cnt = miss_cnt + 1;
            break;
        end
%}
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
newline
fprintf("Mean of Total distance of Drone(m)");
mean(dr_dist(2:size(dr_dist)))

% Mean of Error Distance
fprintf("Mean of Error distance(m)");
mean(de(2:size(de)))

%% Plot
% UAV Trajectory
figure(1);
xlabel('X Position', FontSize = 14);
ylabel('Y Position', FontSize = 14);
plot3(l_DR(:,1), l_DR(:,2), l_DR(:,3), 'k-'); hold on;  % Drone Trajectory
plot3(est_T(2:size(est_T),1), est_T(2:size(est_T),2), est_T(2:size(est_T),3), 'b:o'); hold on;   % Estimated Location of the target
plot3(l_TG(1), l_TG(2), l_TG(3), 'r*');   % Target Location

legend('UAV Trajectory', 'Estimated Target Location', 'Target Location', FontSize = 14);
