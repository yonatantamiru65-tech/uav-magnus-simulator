function Wi_des = X4_input_mixer_ESC_CAN(input)


    global Mx Kv_motor Kv_motor_rad U_sat 
    global Closed_Loop_ESC_wMin Closed_Loop_ESC_wMax

    Wi_max = Kv_motor_rad*U_sat ;
    RPM_ref_1 = input(1) ;
    RPM_ref_2 = input(2) ;
    RPM_ref_3 = input(3) ;
    RPM_ref_4 = input(4) ;

    U_bat = input(5) ;

    W1_des = PWM_data_map(0.055, 1.8961e+03, Closed_Loop_ESC_wMin*2*pi/60, Closed_Loop_ESC_wMax*2*pi/60, RPM_ref_1) ;
    W2_des = PWM_data_map(0.055, 1.8961e+03, Closed_Loop_ESC_wMin*2*pi/60, Closed_Loop_ESC_wMax*2*pi/60, RPM_ref_2) ;
    W3_des = PWM_data_map(0.055, 1.8961e+03, Closed_Loop_ESC_wMin*2*pi/60, Closed_Loop_ESC_wMax*2*pi/60, RPM_ref_3) ;
    W4_des = PWM_data_map(0.055, 1.8961e+03, Closed_Loop_ESC_wMin*2*pi/60, Closed_Loop_ESC_wMax*2*pi/60, RPM_ref_4) ;

%     W1_des = PWM_data_map(1000, 2000, 0, Wi_max, input(1)) ;
%     W2_des = PWM_data_map(1000, 2000, 0, Wi_max, input(2)) ;
%     W3_des = PWM_data_map(1000, 2000, 0, Wi_max, input(3)) ;
%     W4_des = PWM_data_map(1000, 2000, 0, Wi_max, input(4)) ;

    Wi_des = [W1_des W2_des W3_des W4_des] ;
end

