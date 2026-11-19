function Va= App_wind(input)
vx_wind = input(1);
vy_wind = input(2);
vz_wind = input(3);
vx = input(4);
vy = input(5);
vz = input(6);
q0 = input(7) ; 
q1 = input(8) ;
q2 = input(9) ;
q3 = input(10) ;
R_matrix = [(q0^2 + q1^2 - q2^2 - q3^2)  2*(q1*q2 - q0*q3) 2*(q1*q3 + q0*q2) ;
                 2*(q1*q2 + q0*q3)  (q0^2 - q1^2 + q2^2 - q3^2) 2*(q2*q3 - q0*q1) ;
                 2*(q1*q3 - q0*q2) 2*(q2*q3 + q0*q1) (q0^2 - q1^2 - q2^2 + q3^2) ] ;
             
Va_inertial = [vx_wind-vx vy_wind-vy vz_wind-vz]';
Va_local = R_matrix\Va_inertial;
Va = [Va_local(1) Va_local(2) Va_local(3)];
end 