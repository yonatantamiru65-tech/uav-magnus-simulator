function [F_d_local,F_d_global] = Aero_d_initial(Va_int, data_d, quad)
Cx = data_d(1,1);
S_apparente = data_d(2:end,:);

e1 = [1  0  0]' ;
e2 = [0  1  0]' ;
e3 = [0  0  1]' ;

F_d_x_local = (1/2)*Cx*S_apparente(1,1)*abs(Va_int(1))*(Va_int(1))*e1  ;
F_d_y_local = (1/2)*Cx*S_apparente(2,2)*abs(Va_int(2))*(Va_int(2))*e2  ;
F_d_z_local = (1/2)*Cx*S_apparente(3,3)*abs(Va_int(3))*(Va_int(3))*e3  ;


F_d_local = F_d_x_local + F_d_y_local + F_d_z_local ;

q0 = quad(1); q1 = quad(2); q2 = quad(3); q3 = quad(4);
R_matrix = [(q0^2 + q1^2 - q2^2 - q3^2)  2*(q1*q2 - q0*q3) 2*(q1*q3 + q0*q2) ;
                 2*(q1*q2 + q0*q3)  (q0^2 - q1^2 + q2^2 - q3^2) 2*(q2*q3 - q0*q1) ;
                 2*(q1*q3 - q0*q2) 2*(q2*q3 + q0*q1) (q0^2 - q1^2 - q2^2 + q3^2) ] ;
             
F_d_global = R_matrix * F_d_local;
end