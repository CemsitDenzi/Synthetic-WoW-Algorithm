%% Load parameters for WoW   

 %Bounce timerı çıkart. Bounce ederse landing state'e geç
    dt = 0.01;
    t = 0 : dt : 40;
    len = length(t);
    alpha = 0.98;
    std_noise = 0.2;         %Noise standart deviation (std fonksiyonuyla çakışmaması için std_noise yapıldı)
    base_ax = 10;
    base_wheel_speed = 10;
    wait_time = 10/dt;       %Hold spike detection 
    impulse_th = 4.0;        %Threshold for spike detection
    diff_th = 100;           %Speed difference of MLG and NLG;
    landing_start_th = 100;  %landing start altitude threshold
    vertical_speed_th = 50;  %Vertical speed threshold for landing
    ground_alt_th = 5;       %Altitude threshold for detect if vehicle on ground
    wheel_speed_th = 50;     %Wheel speed threshold for detect if wehicle touched or in ground 
    rear_wheel_speed_th = 50;%EKLENDİ: Case 3'te kullanıldığı için tanımlandı
    ground_ver_speed_th = 10;%Vertical speed threshold for detect if vehicle on ground(near zero)
    alpha_th = 12;           %alpha threshold to detect if air craft landed
    alpha_smaller_th = 2;    %To check if airplane is level
    front_debounce_timer = 0;%Timer for NLG debouncing
    debounce_timer = 0;      %Timer for MLG debouncing
    debounce_th = 0.3 / dt; 
    landing_air_speed_th = 60;
    front_debounce_th = 0.3 / dt;
    throttle_toga_th = 10;
    g_th = 0.8; 
    gear_down_locked = 1;
    g = 9.81;
    %struct for spike detection function parameters
    %alpha : HPF coeff, LTA_win_size: Window size for LTA, 
    % STA: Window size for STA, threshold : Threshold for spike detection 
    ax_params = struct('LTA_win_size',10/dt,'STA_win_size', 1/dt,'alpha', 0.98,'threshold',4);
    rear_wheel_params = struct('LTA_win_size',10/dt,'STA_win_size', 1/dt,'alpha', 0.98,'threshold',4);
    front_wheel_params = struct('LTA_win_size',10/dt,'STA_win_size', 1/dt,'alpha', 0.98,'threshold',4);
    dummy_hist = zeros(4, len); % For debugging
    landing_score = 0;