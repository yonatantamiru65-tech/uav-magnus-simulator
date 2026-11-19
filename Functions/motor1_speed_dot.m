function [W_dot I_dot] = motor1_speed_dot(input)

global cQ Jr km R L_mot

    W = input(1) ;
    U = input(2) ;
    W_dot = -(km^2/(Jr*R))*W - (cQ/Jr)*abs(W)*W + (km)/(Jr*R)*U ;


% 
%     FEM = km*W ;
%     Gamma_mot = km*I ;
%     Gamma_c = cQ*abs(W)*W ;
%     I_dot = (1/L)*(U - R*I - FEM) ;  
% 	W_dot = (1/Jr)*(Gama_mot - Gamma_c) ; 
%     
%     I = (1/km)*motor_torque ;
end

