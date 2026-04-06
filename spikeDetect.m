function [spikeDetected,ratio] = spikeDetect(params,current_val,key)

% Input parameters
%params: Struct that contaion "LTA_win_size","STA_win_size","alpha" and "threshold" members
%current_val : Current signal value 
%key : ID given by user for tracking different signal


persistent persist_struct;

if isempty(persist_struct)
    persist_struct = struct;
end


if ~(isfield(persist_struct, key))

    persist_struct.(key).filtered_val_0 = 0;
    persist_struct.(key).prev_val = current_val;
    persist_struct.(key).STA_arr = zeros(1,params.STA_win_size);
    persist_struct.(key).LTA_arr = zeros(1,params.LTA_win_size);

end

alpha = params.alpha; % Highpass filter coeff. 

% First order high-pass
filtered_val = alpha * persist_struct.(key).filtered_val_0 + ...
    alpha * (current_val - persist_struct.(key).prev_val);

persist_struct.(key).prev_val = current_val;
persist_struct.(key).filtered_val_0 = filtered_val;

en_STA = mean(persist_struct.(key).STA_arr.^2);
en_LTA = mean(persist_struct.(key).LTA_arr.^2);

epsilon = 1e-6;
ratio   = en_STA / (en_LTA + epsilon);

persist_struct.(key).STA_arr(1 : end - 1) = persist_struct.(key).STA_arr(2 : end);
persist_struct.(key).LTA_arr(1 : end - 1) = persist_struct.(key).LTA_arr(2 : end);

% Update STA and LTA arrays with the new filtered value
persist_struct.(key).STA_arr(end) = filtered_val;
persist_struct.(key).LTA_arr(end) = filtered_val;

spikeDetected = ratio > params.threshold;


end