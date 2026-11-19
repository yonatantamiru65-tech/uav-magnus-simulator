function Ui = X4_input_mixer(input)
    global Mx Kv_motor Kv_motor_rad U_sat 
    
    PWM1 = input(1) ;
    PWM2 = input(2) ;
    PWM3 = input(3) ;
    PWM4 = input(4) ;

    U_bat = input(5) ;

    U1 = PWM_data_map(1000, 2000, 0, U_bat, PWM1) ;
    U2 = PWM_data_map(1000, 2000, 0, U_bat, PWM2) ;
    U3 = PWM_data_map(1000, 2000, 0, U_bat, PWM3) ;
    U4 = PWM_data_map(1000, 2000, 0, U_bat, PWM4) ;

    Ui = [U1 U2 U3 U4] ;
end


