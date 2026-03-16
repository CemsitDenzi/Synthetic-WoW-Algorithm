    clc; clear all; close all;
    %NOTE: Barometric altitude used. 
    loadParams % call parameters file 
    testDatas % Load test Datas 
    
    % Note : 3 point landing koşuluna dikkat 
    % Ağırlık katsayıları şartlara göre değişiyor ör. terrain durumunda
    % barometrik irtifa katsayısı azalır. Buzlanma durumunda wheel sensor
    % katsayısını düşür etc.

   %Buzlanmaya durumu için wheel speed farkı yerine iki tekeri ayrı ayrı
   %incele

   %stateler arası histerezis ekle yoksa sürekli switching olur.
   %Bonus sinerjilere odaklan ör wheel ve ax'in aynı anda spike yapması
   %gibi 
    %% Main  Loop
for i = 2 : len
    
    [ax_spike(i),dummy_hist(1,i)] = spikeDetect(ax_params,ax(i),"Ax_spike");
    [rear_wheel_acc_spike(i),dummy_hist(2,i)]  = spikeDetect(rear_wheel_params,rear_wheel_acc(i),"Rear_wheel_acc_spike");
    [front_wheel_acc_spike(i),dummy_hist(3,i)]  = spikeDetect(front_wheel_params,front_wheel_acc(i),"front_wheel_acc_spike");
    total_g = sqrt(ax(i)^2 + ay(i)^2 + az(i)^2)/g;

    switch(state)
        case 0 %Initialize
                % Not: Burada sistemin ilk başlatılması ya da havada sistemin
                % resetlenmesi durumunda hangi durumda olduğunun tespit
                % edilmesi amaçlanmıştır. Kalkış, iniş, touchdown gibi ara
                % durumlarda reset işleminin yapılmayacağı varsayılmıştır. 
                % Detect whether system is in air or ground 
                %alpha şartı değişebilir 
                % NOTE :check vertical speed, check throttle ,
            if ((rear_wheel_speed(i) > wheel_speed_th) || (front_wheel_speed(i) > wheel_speed_th)) && abs(vertical_speed(i)) < ground_ver_speed_th
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
            
            if gear_down_locked == 0 || throttle(i) > throttle_toga_th
                landing_score = 0;
            else
                landing_score = 0;

                % Belki wheel speed thresholdu gelebilir
                %flat landing durumunda alpha ve airspeed şartı
                %sağlanmayabilir. Buraya engine RPM eklenecek ama throttle
                %stickten farkı ne 
                landing_conditions = [
                                0.35 * (altitude(i) < landing_start_th), ...
                                0.25 * (vertical_speed(i) < -ground_ver_speed_th), ...
                                0.2 * (air_speed(i) < landing_air_speed_th), ...
                                0.3 * (alpha_ang(i) > alpha_th && total_g < g_th)];
                landing_score = sum(landing_conditions);

            end

            if landing_score >= 0.70 
                state = 2;
            end

        case 2   % Landing Start
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
            
            toga_initiated = (throttle(i) > throttle_toga_th); % Check for go around;
            
            %hard logic
            if (gear_down_locked == 0) || toga_initiated || (altitude(i) > landing_start_th && vertical_speed(i) > vertical_speed_th)
                state = 1;

             % fuzzy logic
            else
                touchdown_score = 0;
                
                touchdown_conditions = [
                    0.35 * (rear_wheel_acc_spike(i)) , ...
                    0.35 * (ax_spike(i)), ...
                    0.20 * (rear_wheel_acc_spike(i) && ax_spike(i)), ... 
                    0.20 * (vertical_speed(i) < -ground_ver_speed_th), ... 
                    0.20 * (rear_wheel_speed(i) > wheel_speed_th), ... 
                    ];

                touchdown_score = sum(touchdown_conditions);

            end

        case 3  %Touchdown of MLG
            toga_initiated = (throttle(i) > throttle_toga_th) && (pitch_rate(i) > -2.0); %Check for touch and go
            
            if toga_initiated
                debounce_timer = 0; 
                state = 1; 
            elseif vertical_speed(i) > ground_ver_speed_th && altitude(i) > ground_alt_th
                debounce_timer = 0;
                state = 2; 
            else
                ground_score = 0;
                flare_ground_score = 0 ;
                flat_ground_score =  0; 


                %Throttle ve pitch rate sarti eklenebilir
                ground_conditions = [
                    0.35 *  (altitude(i) < ground_alt_th), ... 
                    0.30 *  (rear_wheel_speed(i) > 30), ... 
                    0.25 *  (abs(vertical_speed(i)) < 5), ...
                    ];
                
                ground_score = sum(ground_conditions);
                flare_ground_score = ground_score;
                flat_ground_score = ground_score;

                %altta flare - 3 point ayrımı var kontrol et
                if (rear_wheel_speed(i) > 30 && front_wheel_speed(i) < 5 )%flare landing
                    flare_ground_score = ground_score + 0.20;
                elseif abs(rear_wheel_speed(i) - front_wheel_speed(i)) < small_speed_th && rear_wheel_speed(i) > 30 && front_wheel_speed(i) > 30; %three point landing 
                    flat_ground_score = ground_score + 0.20;
                end

                if pitch_rate(i) < -2 
                    flare_ground_score = flare_ground_score + 0.2;
                end

                if alpha < 0.2 
                    flat_ground_score = flat_ground_score + 0.2;
                end 

                

                if flat_ground_score >= 0.8 || flare_ground_score >= 0.8;

                    %timer for bounce
                    debounce_timer = debounce_timer + 1;
                    if debounce_timer >= debounce_th
                        if flare_ground_score > ground_score; 
                            state = 6;
                        else
                            state = 4;   %three point 
                        end                   
                    end
                else
                    debounce_timer = max(0, debounce_timer - 1);
                end
            end

        % Buradan sonrası düzenlenecek
        case 4  % On-Ground
            %Vertical speed artmaya başlarsa ve altitude da ufak bir eşiği
            %geçerse take off stateine geçecek. 
            % WoW = 1 ( yerde) 
            
            liftoff_score = 0;
            
            if altitude(i) > ground_alt_th
                liftoff_score = liftoff_score + 0.35;
            end
            if vertical_speed(i) > ground_ver_speed_th
                liftoff_score = liftoff_score + 0.30;
            end
            if pitch_rate(i) > 0 && alpha(i) > alpha_th
                liftoff_score = liftoff_score + 0.15;
            end
            if air_speed(i) > air_speed(i-1)
                liftoff_score = liftoff_score + 0.10;
            end
            
            %th degiscek
            if liftoff_score >= 0.60
                if throttle(i) > throttle_toga_th
                    state = 5;
                else
                    state = 2; 
                end
            end

        case 6 %Nose up landing (Only MLG's active)
            if (pitch_rate(i) > -1 && pitch_rate(i) < 1) && alpha_ang(i) < alpha_smaller_th
                front_debounce_timer = front_debounce_timer + 1;
                if front_debounce_timer >= front_debounce_th
                    state = 4;
                end
            else
                front_debounce_timer = max(0, front_debounce_timer - 1);
                if vertical_speed(i) > ground_ver_speed_th && altitude(i) > ground_alt_th
                    state = 2; 
                end
            end


        %Will be fixed
        case 5 % Take off start
            if altitude(i) > landing_start_th
                state = 1;
            end
            
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