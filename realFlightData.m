clc;clear all;close all;
load("flightdata.mat")

ax_data = LONG.data;


%Fix data 
for i = 1 : length (ax_data)
    if ax_data(i) > -1.1 && ax_data(i) < -1 
        ax_data(i) = ax_data(i-1);
    end
end

ax_rate =  LONG.Rate; 
alt_data = ALT.data;
alt_rate = ALT.Rate;

wow_data = WOW.data;
wow_rate = WOW.Rate;

fan_speed_data = N1_1.data;
fan_speed_rate = N1_1.Rate;

take_off_step = find(wow_data,1,'first');

pitch_rate = PTCH.Rate;
pitch_data = PTCH.data;

dt_pitch = 1 / pitch_rate;t_pitch  = (0 : length(pitch_data) - 1)' * dt_pitch;

dt_ax  = 1 / ax_rate;  t_ax  = (0 : length(ax_data) - 1)' * dt_ax;
dt_alt  = 1 / alt_rate;  t_alt  = (0 : length(alt_data) - 1)' * dt_alt;
dt_wow  = 1 / wow_rate;  t_wow  = (0 : length(wow_data) - 1)' * dt_wow;
dt_fan_speed  = 1 / fan_speed_rate;  t_fan_speed  = (0 : length(fan_speed_data) - 1)' * dt_fan_speed;
dt_pitch  = 1 / pitch_rate;  t_pitch  = (0 : length(pitch_data) - 1)' * dt_pitch;

vertical_speed_data = zeros(size(alt_data));
filtered_vertical_speed_data = zeros(size(alt_data));
filtered_pitch_rate = zeros(size(pitch_data));
vertical_speed_data(2:end) = diff(alt_data) ./ dt_alt;
vertical_speed_data(1) = vertical_speed_data(2);
LP_alpha = 0.01;

pitch_rate = zeros(size(pitch_data));
pitch_rate(2:end) = diff(pitch_data)./PTCH.Rate;
pitch_rate(1) = pitch_rate(2);


%Lowpass for vertical speed;
for i = 2 : length(vertical_speed_data)
    filtered_vertical_speed_data(i) = LP_alpha * vertical_speed_data(i) + (1 - LP_alpha) * filtered_vertical_speed_data(i -1 );
end

for i = 2 : length(pitch_data)
    filtered_pitch_rate(i) = 0.2 * pitch_rate(i) + (1 - 0.2) * filtered_pitch_rate(i -1 );
end


ax_params = struct('LTA_win_size',500/dt_ax,'STA_win_size', 100/dt_ax,'alpha', 0.88,'threshold',3.5);


rear_wheel_speed = zeros(size(t_alt));
front_wheel_speed = zeros(size(t_alt));



dummy = zeros(size(ax_data));
dummy2 = zeros(size(ax_data));
for i = 1 : length(t_ax)
    [dummy(i), dummy2(i)] = spikeDetect(ax_params,ax_data(i),"AX");
    if i < 1000
        dummy(i) = 0 ;
    end
end

take_off_idx = find(dummy,1,"first");
landing_idx = find(dummy , 1 , "last");
front_wheel_delay = 500;%assumed front wheel touched after rear wheels with some delay

front_wheel_speed(1 : take_off_idx) = 350; % rpm
front_wheel_speed(take_off_idx : landing_idx + front_wheel_delay) = 50;
front_wheel_speed(landing_idx + front_wheel_delay: end ) = 350;

rear_wheel_speed(1 : take_off_idx) = 350; % rpm
rear_wheel_speed(take_off_idx : landing_idx) = 50;
rear_wheel_speed(landing_idx : end ) = 350;


%%  Main loop 



%%
figure(1)

subplot(5,1,1)
plot(t_ax, ax_data);
title("Ax")
subplot(5,1,2)
plot(t_alt, alt_data)
title("Altitude")
subplot(5,1,3)
plot(t_wow, wow_data)
title("Wow")
subplot(5,1,4)
plot(t_alt, filtered_vertical_speed_data)
title("Vertical Speed")
subplot(5,1,5)
plot(t_fan_speed, fan_speed_data)
title("Fan speed ")

figure(2);plot(t_ax,dummy2)

figure(3);plot(t_alt,dummy)
figure(4);plot(t_alt,front_wheel_speed)
figure(5);plot(PTCH.data);
figure(6);plot(filtered_pitch_rate);






