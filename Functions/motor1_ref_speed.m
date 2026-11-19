function ref_speed = motor1_ref_speed(input) 

global cR Jr km R Kv_motor Kv_motor_rad U_sat
    
    U1 = input ;
    
    W1_max = U_sat*Kv_motor_rad ;
    ref_speed = PWM_data_map(0, 12, 0, W1_max, U1) ;
   
end

