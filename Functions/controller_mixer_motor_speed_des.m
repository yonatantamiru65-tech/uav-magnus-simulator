function W_mot_des = controller_mixer_motor_speed_des(input)
    global Mx Kv_motor Kv_motor_rad U_sat Mx_controller_mixer g m
        
    W_mot_des = sqrt(abs(input)) ;
    
end

