function state_dot = Full_state_dot(input)

global Kv_motor Kv_motor_rad Mx J km R Jr cT cQ l lp m Ix Iy Iz U_sat g psi_ref K_pos_speed 
global rho_ground rotor_radius hauteur_rotor
global with_flapping  with_ground_effect Cx rho_air S_apparente rotor_position_mat 
 

vx = input(1)    ;
vy = input(2)    ;
vz = input(3)    ;
phi = input(4)   ;
theta = input(5) ;
psi = input(6)   ;
p = input(7)     ;
q = input(8)     ;
r = input(9)     ;
 
W1 = input(10)   ;
W2 = input(11)   ;
W3 = input(12)   ;
W4 = input(13)   ;

Gamma_p_localDist = input(14) ;
Gamma_q_localDist = input(15) ;
Gamma_r_localDist = input(16) ;

% Gamma_p_globalDist = input(18) ;
% Gamma_q_globalDist = input(19) ;
% Gamma_r_globalDist = input(20) ;
Force_globalDist_x = input(17) ;
Force_globalDist_y = input(18) ;
Force_globalDist_z = input(19) ;
Force_globalDist = [Force_globalDist_x Force_globalDist_y Force_globalDist_z]' ;

 
Force_localDist_x = input(20) ;
Force_localDist_y = input(21) ;
Force_localDist_z = input(22) ;
Force_localDist = [Force_localDist_x Force_localDist_y Force_localDist_z]' ;


vx_wind = input(23) ;
vy_wind = input(24) ;
vz_wind = input(25) ;

 
%%%%%%%%%%%%%%%%%%%%%%%%%% INNER MIXER %%%%%%%%%%%%%%%%%%%%%%%

Wi = [W1 W2 W3 W4]' ; 

chord_prop = 1.6*1e-2  ;        % [m] propeller chord
a_coef = 6 ;                    % [-] lift slope coefficient
% I_b = 1.345*1e-6    ;           % [kg.m^2] blade moment of inertia
% I_m = 2.866*1e-7/2  ;           % [kg.m^2] motor moment of inertia
% T_rotor = 90*(1e-3)/3         % [s] t-constant of prop. rotation rate
n_pale = 3 ;                    % [-] number of blades
rotor_radius = 6.0*1e-2  ;      % [m] blade length
rotor_direction = [+1 -1 +1 -1] ;

theta_0 = 24*pi/180  ;          % [deg] angle of attack at the rotor shaft
theta_1 = -10*pi/180 ;          % [deg] equivalent twist angle

% # [-] solidity ratio
my_sigma = n_pale*chord_prop/(pi*rotor_radius) ;

% # [-] number of Lock
gamma = rho_air*a_coef*chord_prop*rotor_radius^4/Jr ;

rotor_velocity = [W1 W2 W3 W4] ;
rotor_vel = rotor_velocity.*rotor_direction ;


v_h = zeros(3, 4) ;
mu = zeros(4,1) ;


R_matrix = rotation_matrix_from_Euler_123([phi theta psi]) ;
Omega = [p q r]' ;

% FLAPPING PERVIOUS VERSION
% v_h[:, j] = np.matmul(rot_matrix.T, linear_velocity)[:, 0]
% j = 1 ;
% % Version code
% v_h(:, j) = R_matrix'*[vx vy  vz]' + cross(rotor_position_mat(j,:)', [vx vy  vz]') ;
% mu(j) = norm(v_h(1:2, j))/(rotor_radius*rotor_vel(j)) ;
% beta = atan2(v_h(2, j), v_h(1, j)) - psi ;
% a1_mu = ((8/3*theta_0+2*theta_1)*mu(j)-cT)/(1-mu(j)^2/2) ;
% a1_omega = (-16*Omega(2)/(gamma*rotor_vel(j)) + Omega(1)/rotor_vel(j))/(1-mu(j)^2/2) ;
% b1_mu = (4/3)/(1+mu(j)^2/2)*(2*cT*gamma*mu(j)/(3*a_coef*my_sigma)+mu(j)) ;
% b1_omega = ((-16*Omega(1))/(gamma*rotor_vel(j)) - Omega(2)/rotor_vel(j))/(1+ mu(j)^2/2) ;
% a1_s = (a1_mu + a1_omega) ;
% b1_s = (b1_mu + b1_omega) ;
% R1_flap = rot_z(beta)*rot_y(a1_s)*rot_x(b1_s) ;
% 
% 
% j = 2 ;
% % Version code
% v_h(:, j) = R_matrix'*[vx vy  vz]' + cross(rotor_position_mat(j,:)', [vx vy  vz]') ;
% mu(j) = norm(v_h(1:2, j))/(rotor_radius*rotor_vel(j)) ;
% beta = atan2(v_h(2, j), v_h(1, j)) - psi ;
% a1_mu = ((8/3*theta_0+2*theta_1)*mu(j)-cT)/(1-mu(j)^2/2) ;
% a1_omega = (-16*Omega(2)/(gamma*rotor_vel(j)) + Omega(1)/rotor_vel(j))/(1-mu(j)^2/2) ;
% b1_mu = (4/3)/(1+mu(j)^2/2)*(2*cT*gamma*mu(j)/(3*a_coef*my_sigma)+mu(j)) ;
% b1_omega = ((-16*Omega(1))/(gamma*rotor_vel(j)) - Omega(2)/rotor_vel(j))/(1+ mu(j)^2/2) ;
% a1_s = (a1_mu + a1_omega) ;
% b1_s = (b1_mu + b1_omega) ;
% R2_flap = rot_z(beta)*rot_y(a1_s)*rot_x(b1_s) ;
% 
% 
% j = 3 ;
% % Version code
% v_h(:, j) = R_matrix'*[vx vy  vz]' + cross(rotor_position_mat(j,:)', [vx vy  vz]') ;
% mu(j) = norm(v_h(1:2, j))/(rotor_radius*rotor_vel(j)) ;
% beta = atan2(v_h(2, j), v_h(1, j)) - psi ;
% a1_mu = ((8/3*theta_0+2*theta_1)*mu(j)-cT)/(1-mu(j)^2/2) ;
% a1_omega = (-16*Omega(2)/(gamma*rotor_vel(j)) + Omega(1)/rotor_vel(j))/(1-mu(j)^2/2) ;
% b1_mu = (4/3)/(1+mu(j)^2/2)*(2*cT*gamma*mu(j)/(3*a_coef*my_sigma)+mu(j)) ;
% b1_omega = ((-16*Omega(1))/(gamma*rotor_vel(j)) - Omega(2)/rotor_vel(j))/(1+ mu(j)^2/2) ;
% a1_s = (a1_mu + a1_omega) ;
% b1_s = (b1_mu + b1_omega) ;
% R3_flap = rot_z(beta)*rot_y(a1_s)*rot_x(b1_s) ;
% 
% 
% j = 4 ;
% % Version code
% v_h(:, j) = R_matrix'*[vx vy  vz]' + cross(rotor_position_mat(j,:)', [vx vy  vz]') ;
% mu(j) = norm(v_h(1:2, j))/(rotor_radius*rotor_vel(j)) ;
% beta = atan2(v_h(2, j), v_h(1, j)) - psi ;
% a1_mu = ((8/3*theta_0+2*theta_1)*mu(j)-cT)/(1-mu(j)^2/2) ;
% a1_omega = (-16*Omega(2)/(gamma*rotor_vel(j)) + Omega(1)/rotor_vel(j))/(1-mu(j)^2/2) ;
% b1_mu = (4/3)/(1+mu(j)^2/2)*(2*cT*gamma*mu(j)/(3*a_coef*my_sigma)+mu(j)) ;
% b1_omega = ((-16*Omega(1))/(gamma*rotor_vel(j)) - Omega(2)/rotor_vel(j))/(1+ mu(j)^2/2) ;
% a1_s = (a1_mu + a1_omega) ;
% b1_s = (b1_mu + b1_omega) ;
% R4_flap = rot_z(beta)*rot_y(a1_s)*rot_x(b1_s) ;

% FLAPPING NEW VERSION
R1_flap = eye(3) ;
R2_flap = eye(3) ;
R3_flap = eye(3) ;
R4_flap = eye(3) ;

% R1_flap
% R2_flap
% R3_flap
% R4_flap

% Repère local 
b1 = [1  0  0]' ;
b2 = [0  1  0]' ;
b3 = [0  0  1]' ;

e1 = [1  0  0]' ;
e2 = [0  1  0]' ;
e3 = [0  0  1]' ;

F1 = R1_flap*cT*W1^2*b3 ;
F2 = R2_flap*cT*W2^2*b3 ;
F3 = R3_flap*cT*W3^2*b3 ;
F4 = R4_flap*cT*W4^2*b3 ;


% Command_2 = inv(Mx)*[W1^2 W2^2 W3^2 W4^2]' ;
% Gamma_yaw = Command_2(4) ;  
Gamma_yaw = (cQ*W1^2 - cQ*W2^2 + cQ*W3^2 - cQ*W4^2)*b3 ;
Gamma_c_int = cross(rotor_position_mat(1,:)', F1) + cross(rotor_position_mat(2,:)', F2) + cross(rotor_position_mat(3,:)', F3) + cross(rotor_position_mat(4,:)', F4) ;
Gamma_gyro = Jr*cross(Omega, b3)*(-1)*(W1 - W2 + W3 - W4) ; 
Gamma_c = Gamma_c_int + Gamma_yaw + [Gamma_p_localDist Gamma_q_localDist Gamma_r_localDist]' + Gamma_gyro;
Thrust = (F1 + F2 + F3 + F4) ;





%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BOUCLE PQR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gamma_c = [Gamma_p Gamma_q Gamma_r]' ; 

Omega = [p q r ]' ;
Omega_skew = [0 -r q ;
              r 0 -p ;
             -q p 0] ;

Omega_dot = -inv(J)*Omega_skew*J*Omega + inv(J)*Gamma_c ;
p_dot = Omega_dot(1) ;
q_dot = Omega_dot(2) ;
r_dot = Omega_dot(3) ;




%%%%%%%%%%%%%%%%%%%%%%%%% BOUCLE ATTITUDE %%%%%%%%%%%%%%%%%%%%%

phi_dot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r ;
theta_dot = cos(phi)*q - sin(phi)*r ;
psi_dot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r ;

Euler_123_dot = [phi_dot theta_dot psi_dot]' ;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BOUCLE POS SPEED %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Projected_apparent_wind = inv(R_matrix)*[vx+vx_wind vy+vy_wind vz+vz_wind]' ;

F_wind_x_local = (1/2)*Cx*S_apparente(1,1)*R_matrix'*abs(vx+vx_wind)*(vx+vx_wind)*e1  ;
F_wind_y_local = (1/2)*Cx*S_apparente(2,2)*R_matrix'*abs(vy+vy_wind)*(vy+vy_wind)*e2  ;
F_wind_z_local = (1/2)*Cx*S_apparente(3,3)*R_matrix'*abs(vz+vz_wind)*(vz+vz_wind)*e3  ;

F_wind_local = F_wind_x_local + F_wind_y_local + F_wind_z_local ;

R_matrix = rotation_matrix_from_Euler_123([phi theta psi]) ;
% accelerations = -g*b3 + R_matrix*(1/m)*Thrust - (1/(2*m))*Cx*rho_air*c*R_matrix*sqrt([vx+vx_wind vy+vy_wind vz+vz_wind]*[vx+vx_wind vy+vy_wind vz+vz_wind]')*[vx+vx_wind vy+vy_wind vz+vz_wind]' + Thrust_globalDist*b3 ;
accelerations = (1/m)*(-g*m*e3 + R_matrix*Thrust - R_matrix*F_wind_local + Force_globalDist + R_matrix*Force_localDist) ;

x_dot_dot = accelerations(1) ;
y_dot_dot = accelerations(2) ;
z_dot_dot = accelerations(3) ;


speed_dot = [x_dot_dot  y_dot_dot  z_dot_dot] ;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OUTPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
state_dot = [x_dot_dot  y_dot_dot  z_dot_dot phi_dot theta_dot psi_dot  p_dot q_dot r_dot] ;


end

