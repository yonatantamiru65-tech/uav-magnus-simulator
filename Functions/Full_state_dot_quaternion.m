function state_dot = Full_state_dot_quaternion_0(input)

global Kv_motor Kv_motor_rad Mx J km R Jr cT cQ l lp m Ix Iy Iz U_sat g psi_ref K_pos_speed 
global rho_ground rotor_radius hauteur_rotor
global with_flapping  with_ground_effect Cx rho_air S_apparente rotor_position_mat 
global JM1 JM2 

vx = input(1);    vy = input(2);    vz = input(3);
phi = input(4);   theta = input(5);  psi = input(6);

p = input(7);     q = input(8);      r = input(9);
Omega = [p q r]';

q0 = input(10); q1 = input(11); q2 = input(12); q3 = input(13);
quaternion = [q0 q1 q2 q3]';

W1 = input(14); W2 = input(15); W3 = input(16); W4 = input(17);

Gamma_p_localDist = input(18);
Gamma_q_localDist = input(19);
Gamma_r_localDist = input(20);

Force_globalDist_x = input(21);
Force_globalDist_y = input(22);
Force_globalDist_z = input(23);
Force_globalDist = [Force_globalDist_x Force_globalDist_y Force_globalDist_z]';

Force_localDist_x = input(24);
Force_localDist_y = input(25);
Force_localDist_z = input(26);
Force_localDist = [Force_localDist_x Force_localDist_y Force_localDist_z]';

vx_wind = input(27); vy_wind = input(28); vz_wind = input(29);

wM1 = input(30);  wM2 = input(31);
wM1_dot = input(32); wM2_dot = input(33);

% -------- constants / bases --------
Wi = [W1 W2 W3 W4]';

R1_flap = eye(3); R2_flap = eye(3); R3_flap = eye(3); R4_flap = eye(3);

b1 = [1 0 0]'; b2 = [0 1 0]'; b3 = [0 0 1]';
e1 = b1;        e2 = b2;        e3 = b3;

F1 = R1_flap*cT*W1^2*b3;
F2 = R2_flap*cT*W2^2*b3;
F3 = R3_flap*cT*W3^2*b3;
F4 = R4_flap*cT*W4^2*b3;

Gamma_yaw = (cQ*W1^2 - cQ*W2^2 + cQ*W3^2 - cQ*W4^2)*b3;
Gamma_c_int = cross(rotor_position_mat(1,:)', F1) + cross(rotor_position_mat(2,:)', F2) + ...
              cross(rotor_position_mat(3,:)', F3) + cross(rotor_position_mat(4,:)', F4);
Gamma_gyro = Jr*cross(Omega, b3)*(-1)*(W1 - W2 + W3 - W4);
Gamma_c = Gamma_c_int + Gamma_yaw + [Gamma_p_localDist Gamma_q_localDist Gamma_r_localDist]' + Gamma_gyro;
Thrust = (F1 + F2 + F3 + F4);

% -------- PQR dynamics --------
Omega = [p q r]';
Omega_skew = [ 0 -r  q;
               r  0 -p;
              -q  p  0];

w_M1 =  wM1*b2;   w_M2 =  wM2*b2;
w_M1_dot = wM1_dot*b2;
w_M2_dot = wM2_dot*b2;

% Inertia must be 3x3
I = diag([Ix Iy Iz]);

W_sigma = JM1*w_M1 + JM2*w_M2;
W = -Omega_skew*W_sigma;

% Avoid inv(I): use backslash
Omega_dot = - I \ (Omega_skew*I*Omega - JM1*w_M1_dot - JM2*w_M2_dot + W) + I \ Gamma_c;

p_dot = Omega_dot(1);
q_dot = Omega_dot(2);
r_dot = Omega_dot(3);

% -------- Attitude kinematics --------
phi_dot   = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
theta_dot =     cos(phi)*q            - sin(phi)*r;
psi_dot   = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;

quaternion_vector_skew = [ 0  -q3  q2;
                           q3   0 -q1;
                          -q2  q1   0];
quaternion_dot = [ -(1/2)*[q1 q2 q3]*Omega;
                    (1/2)*(quaternion_vector_skew + q0*eye(3))*Omega ];
q0_dot = quaternion_dot(1);
q1_dot = quaternion_dot(2);
q2_dot = quaternion_dot(3);
q3_dot = quaternion_dot(4);

% -------- Position / speed --------
R_matrix = [ (q0^2 + q1^2 - q2^2 - q3^2)  2*(q1*q2 - q0*q3)   2*(q1*q3 + q0*q2);
             2*(q1*q2 + q0*q3)           (q0^2 - q1^2 + q2^2 - q3^2)  2*(q2*q3 - q0*q1);
             2*(q1*q3 - q0*q2)           2*(q2*q3 + q0*q1)   (q0^2 - q1^2 - q2^2 + q3^2) ];

% Avoid inv(R_matrix): use backslash
Projected_apparent_wind = R_matrix \ [-vx_wind+vx; -vy_wind+vy; -vz_wind+vz];

sum_wi      = W1^2 + W2^2 + W3^2 + W4^2;
sum_wi_hend = W1   + W2   + W3   + W4;

F_wind_x_local = -0.0122*abs(Projected_apparent_wind(1))*Projected_apparent_wind(1)*e1 ...
                 - 0.0315*sqrt(cT)*Projected_apparent_wind(1)*sum_wi_hend*e1;
F_wind_y_local = -0.0122*abs(Projected_apparent_wind(2))*Projected_apparent_wind(2)*e2 ...
                 - 0.0315*sqrt(cT)*Projected_apparent_wind(2)*sum_wi_hend*e2;
F_wind_z_local = (-1.4397e-04*Projected_apparent_wind(3)*sum_wi_hend*e3) ...
                 - (0.0745*abs(Projected_apparent_wind(3))*Projected_apparent_wind(3)*e3) ...
                 + (0.0074 * 4 * abs(Projected_apparent_wind(1))*Projected_apparent_wind(1)*e3);

F_wind_local = F_wind_x_local + F_wind_y_local + F_wind_z_local;

accelerations = (1/m)*(-g*m*e3 + R_matrix*Thrust + R_matrix*F_wind_local + Force_globalDist + R_matrix*Force_localDist);

x_dot_dot = accelerations(1);
y_dot_dot = accelerations(2);
z_dot_dot = accelerations(3);

% -------- Output (31 elements) --------
state_dot = [ ...
    x_dot_dot y_dot_dot z_dot_dot ...
    phi_dot theta_dot psi_dot ...
    p_dot q_dot r_dot ...
    q0_dot q1_dot q2_dot q3_dot ...
    Thrust' ...
    Projected_apparent_wind' ...
    (R_matrix*F_wind_local)' ...
    (F_wind_local)' ...
    (Gamma_c_int)' ...
    -0.0745*abs(Projected_apparent_wind(3))*Projected_apparent_wind(3) ...
     0.0074 * 4 * abs(Projected_apparent_wind(1))*Projected_apparent_wind(1) ...
    -1.4397e-04*Projected_apparent_wind(3)*sum_wi_hend ...
];

end
