function s_dot = motors_speed_dot(input)

global J cT cR L m Ix Iy Iz U_sat Jr km R 
    Wi = input(1:4) ;
    Ui = input(5:8) ;
 
    Wi_dot = -(km^2/(Jr*R))*Wi - (cR/Jr)*abs(Wi).*Wi  ;
end 

