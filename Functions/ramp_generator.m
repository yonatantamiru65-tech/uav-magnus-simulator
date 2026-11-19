function output_signal = ramp_generator(current_time, a, initial_condition, ramp_slope)
    t_start_delay=6;
    pente_echelon=1;
    x_ref_start=60;
persistent triggered; % Declare 'triggered' as a persistent variable to remember if the ramp is triggered
    persistent start_time;
    if isempty(start_time)
        start_time=0;
    end 
    if isempty(triggered)
        triggered = false; % Initialize as false when the simulation starts
    end
    
    if a == 1 & ~triggered  % Use element-wise logical operator '&'
        start_time = current_time; % Set the start time when 'a' becomes 1 for the first time
        triggered = true; % Set the triggered flag to true to avoid resetting the start time again
        output_signal = [initial_condition 0 0] + [3/t_start_delay 0 0] * max(0, current_time - start_time)+ ...
            -[3/t_start_delay 0 0]*max(0, current_time - (start_time+t_start_delay))+...
            -[3/t_start_delay 0 0]*max(0, current_time - (start_time+(x_ref_start/3+t_start_delay)- t_start_delay))+...
            [3/t_start_delay 0 0]*max(0, current_time - (start_time+(x_ref_start/3+t_start_delay)));
    
    elseif a==0 & ~triggered
        %start_time = 0; % If 'a' is not 1 or it is already triggered, start_time remains 0
        output_signal = [initial_condition 0 0];
    else 
        output_signal = [initial_condition 0 0] + pente_echelon*[3/t_start_delay 0 0] * max(0, current_time - start_time)+ ...
            -[3/t_start_delay 0 0]*max(0, current_time - (start_time+t_start_delay))+...
            -[3/t_start_delay 0 0]*max(0, current_time - (start_time+(x_ref_start/3+t_start_delay)- t_start_delay))+...
            [3/t_start_delay 0 0]*max(0, current_time - (start_time+(x_ref_start/3+t_start_delay)));
    
    end

