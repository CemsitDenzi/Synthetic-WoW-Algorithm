%% Initialization and Parameters
clc; clear all; close all;

%NOTE: Barometric altitude used. 
dt = 0.01;
t = 0 : dt : 40;
len = length(t);
alpha = 0.98;
std = 0.2;               %Noise standart deviation
base_ax = 10;
base_wheel_speed = 10;
wait_time = 10/dt;       %Hold spike detection 
impulse_th = 4.0;        %Threshold for spike detection
diff_th = 100;           %Speed difference of MLG and NLG;
landing_start_th = 100;  %landing start altitude threshold
vertical_speed_th = 50;  %Vertical speed threshold for landing
ground_alt_th = 5;       %Altitude threshold for detect if vehicle on ground
wheel_speed_th = 50;     %Wheel speed threshold for detect if wehicle touched or in ground 
ground_ver_speed_th = 10;%Vertical speed threshold for detect if vehicle on ground(near zero)
alpha_th = 12;           %alpha threshold to detect if air craft landed

alpha_smaller_th = 2;     %To check if airplane is level

front_debounce_timer = 0;%Timer for NLG debouncing
debounce_timer = 0;      %Timer for MLG debouncing
debounce_th = 0.3 / dt; 

g = 9.81;

%struct for spike detection function parameters
%alpha : HPF coeff, LTA_win_size: Window size for LTA, 
% STA: Window size for STA, threshold : Threshold for spike detection 

ax_params = struct('LTA_win_size',10/dt,'STA_win_size', 1/dt,'alpha', 0.98,'threshold',4);
rear_wheel_params = struct('LTA_win_size',10/dt,'STA_win_size', 1/dt,'alpha', 0.98,'threshold',4);
front_wheel_params = struct('LTA_win_size',10/dt,'STA_win_size', 1/dt,'alpha', 0.98,'threshold',4);

dummy_hist = zeros(4, len); % For debugging

%% Test datas (Not: En son başka bir dosyaya aktar)
ax = base_ax * ones(size(t)) + std * randn(size(t));
ax(38/dt + 1 : 38/dt + 5) = base_ax + 5;
ax(38/dt + 6 : 38/dt + 12) = base_ax - 2;

ay = base_ax * ones(size(t)) + std * randn(size(t));

az = base_ax * ones(size(t)) + std * randn(size(t));

altitude = 400 * ones(size(t));
altitude(25/dt : end) = linspace(400, 0, length(25/dt : len));

vertical_speed = 200 * ones(size(t));
vertical_speed(25/dt : end) = linspace(200, 0, length(25/dt : len));

rear_wheel_speed = base_wheel_speed * ones(size(t)) + std * randn(size(t));
% wheel_speed(25/dt + 2 : end) = linspace(base_wheel_speed, 150, length(25/dt + 2 : len));
rear_wheel_speed(38/dt + 1 : 38/dt + 5) = base_wheel_speed + 5;
rear_wheel_speed(38/dt + 6 : 38/dt + 12) = base_wheel_speed - 2;

front_wheel_speed = base_wheel_speed * ones(size(t)) + std * randn(size(t));
% wheel_speed(25/dt + 2 : end) = linspace(base_wheel_speed, 150, length(25/dt + 2 : len));
front_wheel_speed(38/dt + 1 : 38/dt + 5) = base_wheel_speed + 5;
front_wheel_speed(38/dt + 6 : 38/dt + 12) = base_wheel_speed - 2;

rear_wheel_acc = zeros(size(rear_wheel_speed));
rear_wheel_acc(2:end) = diff(rear_wheel_speed)./dt;
rear_wheel_acc(1) = rear_wheel_acc(2);

front_wheel_acc = zeros(size(front_wheel_speed));
front_wheel_acc(2:end) = diff(front_wheel_speed)./dt;
front_wheel_acc(1) = front_wheel_acc(2);

air_speed = 100 * ones(size(t));

rear_wheel_acc_spike = zeros(size(t));
front_wheel_acc_spike = zeros(size(t));
ax_spike        = zeros(size(t));

alpha = 5 * ones(size(t));
alpha(25/dt + 20 : end) = linspace(5, 0, length(25/dt + 20 : len));

state = 1;
state_log = zeros(size(t));

%% Main  Loop
for i = 2 : len
    
    
    %Detect for spikes
    [ax_spike(i),dummy_hist(1,i)] = spikeDetect(ax_params,ax(i),"Ax_spike");
    [rear_wheel_acc_spike(i),dummy_hist(2,i)]  = spikeDetect(rear_wheel_params,rear_wheel_acc(i),"Rear_wheel_acc_spike");
    [front_wheel_acc_spike(i),dummy_hist(2,i)]  = spikeDetect(front_wheel_params,front_wheel_acc(i),"front_wheel_acc_spike");
    total_g = (ax(i)^2 + ay(i)^2 + az(i)^2)/g;

    % 0 = Initialize, 1  = In air,  2 = Landing start, 3 = Touchdown, 4 = On ground
    % 5 = take-off start 6 = Nose up landing

    switch(state)

        case 0 %Initialize 

            % Not: Burada sistemin ilk başlatılması ya da havada sistemin
            % resetlenmesi durumunda hangi durumda olduğunun tespit
            % edilmesi amaçlanmıştır. Kalkış, iniş, touchdown gibi ara
            % durumlarda reset işleminin yapılmayacağı varsayılmıştır. 

            % Detect whether system is in air or ground 
            %alpha şartı değişebilir 

            % NOTE :check vertical speed, check throttle , 
            if altitude(i) < landing_start_th && alpha(i) < alpha_th
                state = 4;
            else
                state = 1;
            end

            


        case 1 % In air   
            %State geçiş şartı çok keskin ek parametreler eklenebilir. 
            %Ya altitude ölçümü giderse ya da pitot buzlanırsa. 
            % Ama pitot buzlanması çok yüksek irtifalarda görülüyor.
            % Altitude bozulursa kısmı için LIDAR, barometrik irtifa ya da
            % radar altimetre gibi 3 ölçüm var ki her birinin yedekliliği
            % var bu sebeple pek olası gelmedi.

            %persistence timer eklenecek

            alt_req = altitude(i) < landing_start_th && vertical_speed(i) < vertical_speed_th;

            % Might be agressive maneuver so check for total g;
            flare_req = total_g < g_th && alpha > alpha_th;
            
            if alt_req || flare_req 
                state = 2;
            end

        case 2 %Landing start 

            % Tümüyle groundda olabilmesi için ilk olarak uçağın touchdown
            % aşamasından geçmesi gerek. Touchdowndan sonra uçak bounce
            % yapabilir, touch and go yapabilir ya da groundda kalabilir
            % touch and go ya da bounce durumu tespit edilirse landing
            % start moduna geç (bu kısım şüpheli)


            % Burada Landing starttan çıkış şartı eklenecek. touchdowndan
            % sonra uçak tekrar uçuşa geçebilir.(Touch and go) 
            % rear_wheel_detect = (altitude(i) < ground_alt_th) && (rear_wheel_acc_spike(i));
            % impact_detect = (altitude(i) < ground_alt_th) && (ax_spike(i));
            
            %Incase of spike can't be detected or false spike detected use
            %both wheel spike and ax spike.except from small lag they must
            %be occur in same time. 
            if (rear_wheel_acc_spike(i) && ax_spike(i))  || (rear_wheel_speed(i) - front_wheel_speed) > diff_th
                state = 3; % Touchdown detected
            elseif altitude(i) > landing_start_th && vertical_speed(i) > vertical_speed_th
                state = 1; % Abort landing 
            end


            % Bu arada go arounda gidebilir bunun için throttle inputuna
            % bak bu state'de iken 

        case 3 %Touchdown of MLG 


            %altitude thresholdu daha düşük olacak diğerlerinden
            ground_safety_quality = [altitude(i) < ground_alt_th, ...
                                     2 * rear_wheel_speed(i) > rear_wheel_speed_th, ... %Wheel condition weighted
                                     abs(vertical_speed(i)) < ground_ver_speed_th, ...
                                     alpha(i) < alpha_th];

            
            %Debounce tespiti için timer yerine vertical speed değerine
            %bakılabilir.

            %If any 3 out of 4 requirements is true then system is on ground
            if sum(ground_safety_quality) >= 3
                debounce_timer = debounce_timer + 1;

                %Wait for debounce to stop 
                if debounce_timer >= debounce_th
                    if (rear_wheel_speed(i) - front_wheel_speed(i)) > diff_th
                        state = 6;
                    else
                        state = 4;
                    end                   
                end
            else
                debounce_timer = max(0, debounce_timer - 1);
                % It can be touch and go so switch landing start state
                if abs(vertical_speed(i)) > ground_ver_speed_th && altitude(i) > ground_alt_th
                    state = 2; 
                end
            end

        case 4 % On ground 
            %Vertical speed artmaya başlarsa ve altitude da ufak bir eşiği
            %geçerse take off stateine geçecek. 
            % WoW = 1 ( yerde) 


        case 6 %Nose up landing (Only MLG's active)

            if ((front_wheel_acc_spike(i)) || ax_spike(i)) || ((rear_wheel_speed(i) - front_wheel_speed(i) < diff_th && alpha(i) < alpha_th))
                front_debounce_timer = front_debounce_timer + 1;
                if front_debounce_timer >= front_debounce_th
                    state = 4;
                end
            else
                front_debounce_timer = max(0, front_debounce_timer - 1);
                % It can be touch and go so switch landing start state
                if abs(vertical_speed(i)) > ground_ver_speed_th && altitude(i) > ground_alt_th
                    state = 2; 
                end
            end


        case 5 % Take off start 
            
    end
    
    state_log(i) = state;

end

%% Plotting

 plot(t,dummy_hist(2,:))
% plot(t,wheel_acc)

% figure;
% 
% subplot(4,1,1)
% plot(t, ax);
% grid on 
% title('Generated Signal with Noise');
% ylabel('Amplitude');
% 
% subplot(4,1,2)
% plot(t, imu_filtered);
% ylabel('Amplitude');
% title("Filtered Signal");
% grid on;
% 
% subplot(4,1,3)
% plot(t, SL_ratio);
% hold on;
% yline(impulse_th, 'r--', 'Threshold');
% ylabel('Ratio');
% title("STA / LTA Ratio");
% grid on;
% 
% subplot(4,1,4)
% plot(t, state_log, 'LineWidth', 2);
% ylim([0.5 4.5]);
% yticks([1 2 3 4]);
% yticklabels({'In Air', 'Landing Start', 'Touchdown', 'On Ground'});
% xlabel('Time (s)');
% ylabel('State');
% title('State Machine Logic Log');
% grid on;
