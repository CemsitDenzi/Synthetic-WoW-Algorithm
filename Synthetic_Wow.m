clc; clear all; close all;
% NOTE: Barometric altitude used. 
%Pitch rate spike can be useful
% testDatas % Load test Datas 
load("landing.mat")

% Move later
ground_alt_th = 1150;
landing_air_speed_th = 140;
filtered_vertical_speed_th = -5;
throttle_th = 70;

% --- HAFIZA VE SAYAÇ ÖN TANIMLAMALARI ---
dummy_hist = zeros(3, len);
debounce_timer = 0;
front_debounce_timer = 0;
state = 0; % Varsayılan başlangıç durumu
state_log = zeros(size(t));
ax_spike = zeros(size(t));
rear_wheel_acc_spike = zeros(size(t));
front_wheel_acc_spike = zeros(size(t));

%% Main Loop
for i = 2 : len
    
    [ax_spike(i), dummy_hist(1,i)] = spikeDetect(ax_params, ax(i), "Ax_spike");
    [rear_wheel_acc_spike(i), dummy_hist(2,i)]  = spikeDetect(rear_wheel_params, rear_wheel_acc(i), "Rear_wheel_acc_spike");
    [front_wheel_acc_spike(i), dummy_hist(3,i)] = spikeDetect(front_wheel_params, front_wheel_acc(i), "front_wheel_acc_spike");
    
    total_g = sqrt(ax(i)^2 + ay(i)^2 + az(i)^2) / g;
    
    % Radar Altimetre Geçerlilik Kontrolü (Saturasyonu engellemek için)
    ralt_valid = (radio_altitude(i) < ralt_max_range);
    ralt_weight = 0.30 * ralt_valid; % Geçerli değilse 0 çarpanı ile yok sayılır
    
    switch(state)
        case 0 % Initialize
            % Detect whether system is in air or ground 
            if lgup_data(i) == 1
                state = 4;
            else
                state = 1;
            end
            
        case 1 % In air   
            % DÜZELTME: İniş takımı kapalıysa (0) iniş skorunu sıfırla
            if (lgup_data(i) == 0) || throttle(i) > throttle_th
                landing_score = 0;
            else
                landing_score = 0;
                landing_conditions = [
                    0.35 * (altitude(i) < landing_start_th), ...
                    0.25 * (filtered_vertical_speed(i) < filtered_vertical_speed_th), ... 
                    0.20 * (air_speed(i) < landing_air_speed_th), ...
                    0.20 * (alpha_ang(i) > alpha_th && total_g < g_th) %% approach aşamasında direkt flare yapmaz
                ];
                landing_score = sum(landing_conditions);
            end
            
            if landing_score >= 0.70 
                state = 2;
            end
            
        case 2   % Landing Start
            toga_initiated = (throttle(i) > throttle_toga_th);
            
            % İniş takımı kapalıysa (0) pas geç
            if (lgup_data(i) == 0) || toga_initiated || (altitude(i) > landing_start_th && filtered_vertical_speed(i) > ground_ver_speed_th) 
                state = 1;
            else
                touchdown_score = 0; %Wheel spike gözükmüyor kontrol et
                touchdown_conditions = [
                    0.35 * (rear_wheel_acc_spike(i)), ...
                    0.35 * (ax_spike(i)), ...
                    0.20 * (rear_wheel_acc_spike(i) && ax_spike(i)), ... 
                    0.20 * (filtered_vertical_speed(i) < filtered_vertical_speed_th), ... 
                    0.20 * (rear_wheel_speed(i) > rear_wheel_speed_th), ... 
                    ralt_weight * (radio_altitude(i) <= ground_alt_th) % sacma olabililr if kısmında bakılıyor
                ];
                touchdown_score = sum(touchdown_conditions);
                
                if touchdown_score >= 0.75
                    state = 3;
                end
            end
            
        case 3  % Touchdown of MLG
            toga_initiated = ((throttle(i) > throttle_toga_th) && (pitch_rate(i) > pitch_rate_toga_th)); 
            
            if toga_initiated || filtered_vertical_speed(i) > ground_ver_speed_th % touch and go 
                debounce_timer = 0; 
                state = 1; 
            elseif filtered_vertical_speed(i) > ground_ver_speed_th && altitude(i) > ground_alt_th
                debounce_timer = 0;
                state = 2; 
            else
                ground_score = 0;
                ground_conditions = [
                    0.35 * (altitude(i) < ground_alt_th), ... 
                    0.40 * (rear_wheel_speed(i) > rear_wheel_speed_th), ... 
                    0.20 * (abs(filtered_vertical_speed(i)) < ground_ver_speed_th), ... 
                    ralt_weight * (radio_altitude(i) <= ground_alt_th)
                ];
                
                ground_score = sum(ground_conditions);
                flare_score = ground_score;
                flat_score = ground_score;
                
                flare_score = flare_score + 0.2 * (alpha_ang(i) > alpha_th); 
                flat_score = flat_score + 0.2 * (alpha_ang(i) < alpha_smaller_th); 
                
                if flat_score >= 0.80 || flare_score >= 0.80
                    debounce_timer = debounce_timer + 1;
                    if debounce_timer >= debounce_th
                        if flare_score > flat_score || (rear_wheel_speed(i) - front_wheel_speed(i) > diff_th)
                            state = 6;
                        else
                            state = 4;
                        end                   
                    end
                else
                    debounce_timer = max(0, debounce_timer - 1);
                end
            end
            
        case 6 % Nose up landing (Only MLG's active) - flare landing
            if (abs(pitch_rate(i)) < 1) && alpha_ang(i) < alpha_smaller_th
                front_debounce_timer = front_debounce_timer + 1;
                if front_debounce_timer >= front_debounce_th
                    state = 4;
                end
            else
                front_debounce_timer = max(0, front_debounce_timer - 1);
                if filtered_vertical_speed(i) > ground_ver_speed_th && altitude(i) > ground_alt_th
                    state = 2; 
                end
            end
            
        case 4  % On-Ground
            if filtered_vertical_speed(i) > ground_ver_speed_th && air_speed(i) > landing_air_speed_th  && alpha_ang(i) > 0 
                state = 5;
            elseif filtered_vertical_speed(i) > ground_ver_speed_th && altitude(i) > ground_alt_th
                state = 2; 
            end
            
        case 5 % Take off start
            if ax(i) < 0 && throttle(i) < throttle_th && air_speed(i) < landing_air_speed_th
                state = 4;
            elseif altitude(i) > landing_start_th
                state = 1;
            end
            
    end
    
    state_log(i) = state;
    dummy_hist(i);
end

%% Plotting - Durum Analizi
% figure('Name', 'State Machine Log', 'NumberTitle', 'off');
% subplot(2,1,1)
% plot(t, state_log, 'b', 'LineWidth', 1.5);
% title('Weight on Wheels (WoW) State Machine Log');
% ylabel('State (1-6)');
% yticks(1:6);
% grid on;
% 
% subplot(2,1,2)
% plot(t, radio_altitude, 'g', t, ralt_valid * 5000, 'r--');
% title('Radio Altitude Validity');
% legend('Radar Alt', 'Validity Mask');
% ylabel('ft');
% xlabel('Time (s)');
% grid on;