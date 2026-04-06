%% --- BAŞLATMA, VERİ YÜKLEME VE SENKRONİZASYON (Init & Load) ---
clc; clear all; close all;

% 1. Adım: Orijinal Verileri Yükleme ve Gürültü Temizleme
load("flightdata.mat")
g = 9.81;

% Longitudinal g (ax)
ax_raw = LONG.data;
for i = 1 : length(ax_raw)
    if ax_raw(i) > -1.1 && ax_raw(i) < -1 
        ax_raw(i) = ax_raw(i-1) ;
    end
end
dt_ax = 1 / LONG.Rate;  t_ax = (0 : length(ax_raw) - 1)' * dt_ax;

% Lateral g (ay) 
ay_raw = LATG.data;
for i = 2 : length(ay_raw)
    if ay_raw(i) < -0.5 
        ay_raw(i) = ay_raw(i-1);
    end
end
dt_ay = 1 / LATG.Rate;  t_ay = (0 : length(ay_raw) - 1)' * dt_ay;

% Vertical/Normal g (az)
az_raw = VRTG.data;
for i = 2 : length(az_raw)
    if az_raw(i) < -1 
        az_raw(i) = az_raw(i-1);
    end
end
dt_az = 1 / VRTG.Rate;  t_az = (0 : length(az_raw) - 1)' * dt_az;

% İrtifalar (Barometrik ve Radar)
alt_raw = ALT.data;
dt_alt = 1 / ALT.Rate;  t_alt = (0 : length(alt_raw) - 1)' * dt_alt;
ralt_raw = RALT.data;
dt_ralt = 1 / RALT.Rate; t_ralt = (0 : length(ralt_raw) - 1)' * dt_ralt;

% WoW ve LGUP (Ayrık/Lojik Veriler)
wow_raw = WOW.data;
dt_wow = 1 / WOW.Rate;  t_wow = (0 : length(wow_raw) - 1)' * dt_wow;
lgup_raw = LGUP.data;
dt_lgup = 1 / LGUP.Rate; t_lgup = (0 : length(lgup_raw) - 1)' * dt_lgup;

% Motor (N1) ve Aerodinamik Veriler
throttle_raw = N1_1.data;
dt_fan_speed = 1 / N1_1.Rate; t_fan_speed = (0 : length(throttle_raw) - 1)' * dt_fan_speed;
alpha_raw = AOAC.data;
dt_alpha = 1 / AOAC.Rate; t_alpha = (0 : length(alpha_raw) - 1)' * dt_alpha;
air_speed_raw = TAS.data;
dt_tas = 1 / TAS.Rate; t_tas = (0 : length(air_speed_raw) - 1)' * dt_tas;

% Pitch Rate (Türev)
pitch_data = PTCH.data;
dt_pitch = 1 / PTCH.Rate; t_pitch = (0 : length(pitch_data) - 1)' * dt_pitch;
pitch_rate_raw = zeros(size(pitch_data));
pitch_rate_raw(2:end) = diff(pitch_data) ./ dt_pitch;
pitch_rate_raw(1) = pitch_rate_raw(2);

% 2. Adım: Master Time (Ortak Zaman Ekseni) Oluşturma
t_max = min([t_ax(end), t_ay(end), t_az(end), t_alt(end), t_ralt(end), t_wow(end), t_lgup(end), t_fan_speed(end), t_pitch(end), t_alpha(end), t_tas(end)]);
dt_master = 0.01; % Ana çalışma frekansı (100 Hz)
t = (0 : dt_master : t_max)'; 
len = length(t); 

% 3. Adım: İnterpolasyon (Senkronizasyon)
ax             = interp1(t_ax, ax_raw, t, 'linear', 'extrap');
ay             = interp1(t_ay, ay_raw, t, 'linear', 'extrap');
az             = interp1(t_az, az_raw, t, 'linear', 'extrap');
altitude       = interp1(t_alt, alt_raw, t, 'linear', 'extrap');
radio_altitude = interp1(t_ralt, ralt_raw, t, 'linear', 'extrap');
throttle       = interp1(t_fan_speed, throttle_raw, t, 'linear', 'extrap');
pitch_rate     = interp1(t_pitch, pitch_rate_raw, t, 'linear', 'extrap');
alpha_ang      = interp1(t_alpha, alpha_raw, t, 'linear', 'extrap'); 
air_speed      = interp1(t_tas, air_speed_raw, t, 'linear', 'extrap');
wow_data       = interp1(t_wow, wow_raw, t, 'nearest', 'extrap'); 
lgup_data      = interp1(t_lgup, lgup_raw, t, 'nearest', 'extrap'); 

% 4. Adım: İlk Türevlerin Hesaplanması (Kırpmadan ÖNCE yapılmalı)
vertical_speed = zeros(size(altitude));
filtered_vertical_speed = zeros(size(altitude));
vertical_speed(2:end) = diff(altitude) ./ dt_master;
vertical_speed(1) = vertical_speed(2);

LP_alpha = 0.01;
for i = 2 : len
    filtered_vertical_speed(i) = LP_alpha * vertical_speed(i) + (1 - LP_alpha) * filtered_vertical_speed(i -1);
end

% =========================================================================
% 5. Adım: VERİ KIRPMA (CROPPING) - Seyir Uçuşunun Çıkarılması
% =========================================================================
cut_start = 2500; % Kesilecek kısmın başlangıç saniyesi
cut_end   = 6000; % Kesilecek kısmın bitiş saniyesi
keep_idx  = [find(t <= cut_start); find(t >= cut_end)];

% Tüm vektörleri kırp
ax = ax(keep_idx);
ay = ay(keep_idx);
az = az(keep_idx);
altitude = altitude(keep_idx);
radio_altitude = radio_altitude(keep_idx);
throttle = throttle(keep_idx);
pitch_rate = pitch_rate(keep_idx);
alpha_ang = alpha_ang(keep_idx);
air_speed = air_speed(keep_idx);
wow_data = wow_data(keep_idx);
lgup_data = lgup_data(keep_idx);
vertical_speed = vertical_speed(keep_idx);
filtered_vertical_speed = filtered_vertical_speed(keep_idx);

% Kırpılan veriye göre sürekli bir zaman ekseni oluştur
t = (0 : length(ax) - 1)' * dt_master;
len = length(t); 
% =========================================================================

% 6. Adım: Sistem Parametreleri ve Eşik Değerleri 
std_noise = 0.2;         
impulse_th = 4.0;        
g_th = 0.8; 
landing_start_th = 5000; 
ground_alt_th = 20;      
vertical_speed_th = -30; 
ground_ver_speed_th = 5; 
diff_th = 100;           
wheel_speed_th = 150;     
rear_wheel_speed_th = 150;
alpha_th = 8;           
alpha_smaller_th = 2;    
landing_air_speed_th = 180; 
throttle_th = 40;        
throttle_toga_th = 85;   
front_debounce_th = 0.3 / dt_master;
debounce_th = 0.3 / dt_master; 

pitch_rate_toga_th = 0; 
ralt_max_range = 4800;

ax_params          = struct('LTA_win_size', 500/dt_master, 'STA_win_size', 100/dt_master, 'alpha', 0.88, 'threshold', 3.5);
rear_wheel_params  = struct('LTA_win_size', 500/dt_master, 'STA_win_size', 100/dt_master, 'alpha', 0.88, 'threshold', 3.5);
front_wheel_params = struct('LTA_win_size', 500/dt_master, 'STA_win_size', 100/dt_master, 'alpha', 0.88, 'threshold', 3.5);

% 7. Adım: Sentetik Veriler ve Ön Tanımlamalar (Kırpılmış Veri Üzerinden)
dummy = zeros(size(ax));
for i = 1 : len
    [dummy(i), ~] = spikeDetect(ax_params, ax(i), "AX");
    if i < 1000
        dummy(i) = 0 ;
    end
end
take_off_idx = find(dummy, 1, "first");
landing_idx = find(dummy, 1, "last");
front_wheel_delay = 500; 

front_wheel_speed = zeros(size(t));
rear_wheel_speed = zeros(size(t));

front_wheel_speed(1 : take_off_idx) = 350; 
front_wheel_speed(take_off_idx : min(landing_idx + front_wheel_delay, len)) = 50;
if landing_idx + front_wheel_delay < len
    front_wheel_speed(landing_idx + front_wheel_delay : end) = 350;
end

rear_wheel_speed(1 : take_off_idx) = 350; 
rear_wheel_speed(take_off_idx : landing_idx) = 50;
rear_wheel_speed(landing_idx : end) = 350;

rear_wheel_acc = zeros(size(rear_wheel_speed));
rear_wheel_acc(2:end) = diff(rear_wheel_speed) ./ dt_master;
rear_wheel_acc(1) = rear_wheel_acc(2);

front_wheel_acc = zeros(size(front_wheel_speed));
front_wheel_acc(2:end) = diff(front_wheel_speed) ./ dt_master;
front_wheel_acc(1) = front_wheel_acc(2);

% Main Loop İçin Hafıza
dummy_hist = zeros(4, len); 
rear_wheel_acc_spike = zeros(size(t));
front_wheel_acc_spike = zeros(size(t));
ax_spike = zeros(size(t));
state_log = zeros(size(t));
landing_score = 0;

%% Plotting - Detaylı Analiz Grafikleri
figure('Name', 'Uçuş Dinamikleri ve İrtifa', 'NumberTitle', 'off');
subplot(4,1,1)
plot(t, altitude, 'b', 'LineWidth', 1); hold on;
plot(t, radio_altitude, 'g', 'LineWidth', 1);
title('Altitude Analysis (Cruising phase removed)');
ylabel('Altitude (ft)');
legend('Barometric', 'Radar');
grid on;

subplot(4,1,2)
plot(t, filtered_vertical_speed, 'k');
title('Filtered Vertical Speed');
ylabel('Rate (ft/s)');
grid on;

subplot(4,1,3)
plot(t, ax, 'b'); hold on;
plot(t, ay, 'g');
plot(t, az, 'r');
title('Linear Accelerations (Body Axes)');
ylabel('g');
legend('Ax (Long)', 'Ay (Lat)', 'Az (Vert)');
grid on;

subplot(4,1,4)
plot(t, pitch_rate, 'm');
title('Pitch Rate');
ylabel('deg/s');
xlabel('Time (s)');
grid on;

figure('Name', 'Sistem ve Sensör Durumları', 'NumberTitle', 'off');
subplot(4,1,1)
plot(t, throttle, 'Color', [0.85 0.33 0.1]);
title('Engine Fan Speed (N1)');
ylabel('% RPM');
grid on;

subplot(4,1,2)
plot(t, air_speed, 'b');
title('True Air Speed (TAS)');
ylabel('Knots / m/s');
grid on;

subplot(4,1,3)
plot(t, rear_wheel_speed, 'r'); hold on;
plot(t, front_wheel_speed, 'b--');
title('Wheel Speeds (Rear & Front)');
ylabel('RPM');
legend('Rear Wheels', 'Front Wheel');
grid on;

subplot(4,1,4)
yyaxis left
plot(t, lgup_data, 'r', 'LineWidth', 1.5);
ylabel('LGUP (1:UP, 0:DOWN)');
ylim([-0.2 1.2]);
yyaxis right
plot(t, alpha_ang, 'k-');
ylabel('Alpha (deg)');
title('Landing Gear Status & Angle of Attack');
xlabel('Time (s)');
grid on;