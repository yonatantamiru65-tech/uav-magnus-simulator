%% ________________________  UAV X4 SIMULATOR  ____________________________
% -------------------------- SUSBIELLE Pierre -----------------------------
% ------------------------------ GIPSA-Lab --------------------------------

%clear all
close all
clc
addpath codegen\ Functions\ yalmip\

global Kv_motor Kv_motor_rad Mx J km R Jr cT cQ l lp m Ix Iy Iz U_sat g psi_ref K_pos_speed  L_mot
global rho_ground rotor_radius hauteur_rotor
global with_flapping  with_ground_effect Cx rho_air S_apparente rotor_position_mat R_bat
global Mx_inv Mx_noYaw Mx_inv_noYaw
global Closed_Loop_ESC_wMin Closed_Loop_ESC_wMax
global m_M1 m_M2 s_M1D s_M2D JM1 JM2 Jr_Magnus cQ_Magnus
global Ts betai 

Ts = 0.02 ;             % Période d'échantillonage contrôle [s]
g = 9.81 ;              % Accélération de pesanteur [ms.-2]

comp = 0;
wind_turb = 1;
wind = 1;

Bp_offset = 0.06;
%% I. Drone model parameters and simulation parameters 

%%% ____________________ I.1 MODELE DYNAMIQUE  ____________________________


% ------------------------ GEOMETRIE CHASSIS ------------------------------


% _____                               _____
%| 2(-)| --lp_left--  |  --lp_right--| 1(+)|
%|_____|              |              |_____|
%      \              |             /
%       \             |            /
%        \        lp_front        /
%         \           |          /
%          \          |         /
%           \         |        /
%           /         |        \
%          /          |         \
%         /           |          \     
%        /        lp_back         \  
%       /             |            \  
%      /              |             \
%     /               |              \
% _____               |               _____
%| 3(+)|              |              | 4(-)|
%|_____|                             |_____|



% Positions des rotors dans le repère local :
lp_left = (222/2)*1e-3 ;        % Bras de levier -distances projetées- [m]
lp_right = (222/2)*1e-3 ;       % Bras de levier -distances projetées- [m]
lp_front = (170/2)*1e-3 ;       % Bras de levier -distances projetées- [m]
lp_back = (170/2)*1e-3 ;        % Bras de levier -distances projetées- [m]
h_chassis = 0.15 ;              % Hauteur totale chassis [m]
h_rotor = 0.030 ;               % Hauteur plan de rotation moteurs [m]
            
         
rotor_position_mat = [lp_front   -lp_right    h_rotor ;
                        lp_front    lp_left     h_rotor ;
                       -lp_back     lp_left     h_rotor ;
                       -lp_back    -lp_right    h_rotor] ;

m = 1.445; %Qudacopter Mass
%m = 1.6193; %QAadcopter Mass with Magnus rotors 
%m = 1.746; %Total Mass
m = 1.7460;

% ------------------------- Magnus Parameters --------------------------------

DM = 0.055;
RM = DM/2;
LM = 0.28*2;
S = 2*LM*RM;
Slat = pi*RM^2;


w_Mag_per = 0.2;
tau_mag = 0.1;                       %model of actuator for magnus rotation 
tau_mag_model = 0.105;               %"identified one used for control (if used)
CDy = 1;
Cf = 0.003;                          % friction Magnus coefficient between 0.003 and 0.0045
cQ_Magnus = 0.5*S/2*RM^3*Cf;         % Coefficient de trainée Magnus 

s_M1D = [0, LM/4, 0]';
s_M2D = [0, -LM/4, 0]';
m_M1 = 0.049;
m_M2 = 0.049;

JM1 = zeros(3,3);
JM1(1,1) = 1/2*m_M1*RM^2+1/3*m_M1*(LM/2)^2;
JM1(2,2) = m_M1*RM^2;
JM1(3,3) = JM1(1,1);

JM2 = zeros(3,3);
JM2(1,1) = 1/2*m_M2*RM^2+1/3*m_M2*(LM/2)^2;
JM2(2,2) = m_M2*RM^2;
JM2(3,3) = JM2(1,1);

% ------------------------- Winch Parameters --------------------------------

m_w = 0.0481;        % Winch mass
gamma_w = 0.4; 
tau_ur = 0.023;      % for simulation
F_fric=0.5;          % -N the dry friction at the on-ground winch 

% ------------------------- INERTIE TOTALE --------------------------------
%                - ramenée en G, repère local {B} -

%inertia without magnus
Ix = 5686.57/(1000^2) ;     % Moment d'inertie x [-]
Ixy = -0.045/(1000^2) ;
Ixz = -17.253/(1000^2) ;
Iyx = -0.045/(1000^2) ;
Iy = 5413.986/(1000^2) ;     % Moment d'inertie y  [-]
Iyz = -0.774/(1000^2) ;   
Izx = -17.253/(1000^2) ;
Izy = -0.774/(1000^2) ;
Iz = 8946.234/(1000^2) ;     % Moment d'inertie z [-]


%inertia with magnus
Ix = 8192.376/(1000^2) ;     % Moment d'inertie x [-]
Ixy = 0.576/(1000^2) ;
Ixz = -15.479/(1000^2) ;
Iyx = 0.576/(1000^2) ;
Iy = 5442.037/(1000^2) ;     % Moment d'inertie y  [-]
Iyz = -0.011/(1000^2) ;   
Izx = -15.479/(1000^2) ;
Izy = -0.011/(1000^2) ;
Iz = 11453.555/(1000^2) ;     % Moment d'inertie z [-]

J = [Ix Ixy Ixz ;
     Iyx Iy Iyz ;
     Izx Izy Iz] ;                      % Matrice d'inertie [-]



% ---------- Modélisation des Frottements aérodynamiques -----------------
% Dans le repère local : F = (1/2)*Cx*rho_air*S*[vx vy vz].^2 

rho_air = 1.293 ;           % Masse volumique de l'air [kg.m^-3] 
Cx = 1.05 ;                 % Coefficient frottement aero rectangle [-]

S_apparente_front = h_chassis*(lp_left + lp_right) ; 
                            % Surface apparente [m.-2]   
S_apparente_side = h_chassis*(lp_front + lp_back) ;   
                            % Surface apparente [m.-2]   
S_apparente_top = (lp_front + lp_back)*(lp_left + lp_right) ;  
                            % Surface apparente [m.-2]   
S_apparente =  diag([S_apparente_front S_apparente_side S_apparente_top]) ;
                            % Surface apparente [m.-2]   


%%% ________________ I.2 ACTIONNEURS ET PUISSANCE _________________________

% ------------------------- PARAMETRES MOTEUR -----------------------------
% Type de moteur : T-Motor F60 v IV 1750kv
% D'apres l'OUTIL ci-joint et les données constructeur

Kv_motor = 1750 ;                       % Constante moteur [tr.min-1] 
Kv_motor_rad = Kv_motor*2*pi/60 ;       % Constante moteur [rad.s-1]
km = 1/Kv_motor_rad ;

rotor_radius = 6.0*1e-2 ;               % Diamètre moteur [m]
hauteur_rotor = 29.3*1e-3 ;             % Hauteur moteur [m]

Jr = 1.2991*1e-05 ;                     % Moment d'inertie moteur [a definir]
Jr_Magnus=JM1(2,2);                     % Moment of inertia magnus moteur 
R = 0.07 ;                              % Résistance interne [Ohm]
L_mot = 1.0000e-04 ;                    % Inductance interne [H]


% ----------------------- PARAMETRES HELICES -----------------------------

with_flapping = 0 ;                     % Seclect flapping in simulation [0/1]
with_ground_effect = 1 ;                % Seclect ground effect in simulation [0/1]

% Type d'hélice : 5030 GEFMANN.
% Depuis Outil ci-oint et data constructeur: 
cT = 2.4815e-6;                         % Coefficient de poussée [-]
cQ = 2.1113*1e-08 ;                     % Coefficient de trainée [-]
rho_ground = 1.4 ;                      % Effet de sol [-]
cT_1_6 = 2.1030e-06;
                  

% ------------------------- PARAMETRES ESC --------------------------------
% Valeurs réglées pour les ESCs boucle fermée sous AM32 et mixeur par
% défault PX4 : 

CLOSED_LOOP_ESCs = 1 ;                  % Select ESC Configuration [0/1] 

Closed_Loop_ESC_wMin = 3000 ;           % Vitesse moteur minimale [rpm]
Closed_Loop_ESC_wMax = 24000 ;          % Vitesse moteur maximale [rpm]



% ------------------- PARAMETRES BATTERIE [1-6S] --------------------------
nb_battery_cells = 4 ;                  % Nombre de cellule de batterie [-]  
U_sat = 3.7*nb_battery_cells ;          % Tension nominale batterie [V]
R_bat = 0.024;                          % Résistance interne batterie [Ohm]



%%% ------------- Conditions initiales position - attitude ----------------

% intial condition for the main simulations
ri    = 0.490142; %0.5;  
ethai = 0 *pi/180; 
betai = 17.8306*pi/180; %20*pi/180; 

initial_speed_conditions = 0*[1 1 1] ;
initial_attitude_condition = 1*[0 0 pi] ;     % [rad] Attitude initiale 
initial_attitude_quaternion_condition = euler2quaternion(initial_attitude_condition);%to calculate init of quaternions
initial_pos_conditions = [ri*cos(betai) 0 ri*sin(betai) + Bp_offset];% + R_initial * [0; 0 ; Bp_offset];  
state_int = [0 0 0 initial_attitude_condition 0 0 0 initial_attitude_quaternion_condition];


if initial_pos_conditions ~= [0 0 0]
    disp("WARNING : Initial position conditions set to :")
    disp(initial_pos_conditions) 
end
if initial_speed_conditions ~= [0 0 0]
    disp("WARNING : Initial speed conditions set to :")
    disp(initial_speed_conditions) 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Trajectory %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rf    = 1.5; %0.8; %9.51;
ethaf = 25*pi/180;     % for 2D trajectory
%betaf = 40*pi/180;


t0 = 0; 
t1 = t0 + 15;% + 35; 
t2 = t1 + 15;% + 35; 
t3 = t2 + 15;% + 35; 
t4 = t3 + 15;% + 35;
final_time = t4;%seconds - end of simulation

%%% __________________ I.3 AUTOPILOTE - CAPTEURS __________________________
% Autopilote embarqué PX4 - Système de capture de mouvement VICON TRACKER

% -------------------------- Matrice de mix -------------------------------
Mx = [ cT        cT      cT         cT ; 
      -cT*lp_right   cT*lp_left    cT*lp_left   -cT*lp_right ;
      -cT*lp_front   -cT*lp_front    cT*lp_back   cT*lp_back ;
      cQ         -cQ      cQ     -cQ ] ; 

Mx_noYaw = [ cT        cT      cT         cT ; 
            -cT*lp_right   cT*lp_left    cT*lp_left   -cT*lp_right ;
            -cT*lp_front   -cT*lp_front    cT*lp_back   cT*lp_back ] ;

Mx_inv = inv(Mx) ;
Mx_inv_noYaw = pinv(Mx_noYaw) ;


% Vicon Tracer Modelisation 
ViconTracker_noise_variance = 0.000001 ;    % Vicon noise variation [m^2]
ViconTracker_noise_mean = 0.00001 ;         % Vicon noise mean [m]




%% II. Simulation general parameters


%%% ------------------ Vent 3D repère global {E} --------------------------
% VENT CONSTANT DANS LE REPERE GLOBAL
dt = 0.002;

        current_state_P = 1;
        load('wind_data1');
        vwind = datavals(1,1:final_time/dt+1);
        [wind_row, wind_clm] = size(vwind);

        t_winddirc_change = 6; 
        tims = [t_winddirc_change : dt : (t_winddirc_change+1)];
        vx_inter = vwind(1, t_winddirc_change/dt:(t_winddirc_change+1)/dt).*((cos(10*pi/180)-1)*tims+(t_winddirc_change+1-t_winddirc_change*cos(10*pi/180)));
        vy_inter = (sin(10*pi/180)*vwind(1, t_winddirc_change/dt:(t_winddirc_change+1)/dt)).*(tims-t_winddirc_change);
        vx_wind = [vwind(1,1:t_winddirc_change/dt-1) vx_inter vwind(1,(t_winddirc_change+1)/dt +1:final_time/dt+1)*cos(10*pi/180)] ;
        vy_wind = [zeros(1,t_winddirc_change/dt-1) vy_inter vwind(1,(t_winddirc_change+1)/dt +1:final_time/dt+1)*sin(10*pi/180)] ;
        vz_wind = zeros(1,final_time/dt +1) ; 
        if wind_turb == 1
        vmag = mean(vwind); %to determine Magnus rotational speed 
        else 
            vmag = 4;
       
        end
        vitesse_vent = [vx_wind vy_wind vz_wind]' ;
if wind == 0                  % Composante z vent [m.s-1]
        vmag=0;
        vitesse_vent = [0, 0, 0]' ;
end


% 
% vx_wind = 5;                   % Composante x vent [m.s-1]
% vy_wind = 0 ;                   % Composante y vent [m.s-1]
% vz_wind = 0 ;                   % Composante z vent [m.s-1]
% vitesse_vent = [vx_wind vy_wind vz_wind]' ;

% ---------------------- PX4 Attitude Saturation --------------------------
max_angle_deg = 35 ;                % [Degres] Maximum angular reference
max_angle = max_angle_deg*pi/180 ;  % [rad] 


%%% ------------- FL Tuning ----------------
% Control Compensation
tau_ur_model = 0.001; % "idnetified" one, for compensation
tau_ue_model = 0.126;
tau_ub_model = 0.1;


% FL control Tuning

wn_r    = 1.5; %2;%1.8;
wn_beta = 1.5; %2;%1.2;
wn_etha = 1;

% wn_r    = 1.8; %2;%1.8;
% wn_beta = 1.6; %2;%1.2;
% wn_etha = 1.1;

wn_r    = 2; %2;%1.8;
wn_beta = 2; %2;%1.2;
wn_etha = 2;

T0 = 1.5;       %desired Tether tension

%%% ------------- Initial Guess of CA optimisation problem ----------------
% 


% X1 = 0:0.01:8;
% Va = vx_wind; 
% % BED MODEL 
% CL = (1 - 1./(1 + exp(-0.904 * X1))).*(0.0276*X1.^4+4.6595*X1.^3-6.0369*X1.^2+2.9557*X1) +  1./(1 + exp(-0.904 * X1)) .*(1.0849*X1);
% CD = -0.0461 * X1.^3 + 0.5167 * X1.^2 -0.6511 * X1 +0.9081;
% 
% FL = 0.5 * rho_air * S * CL.* Va^2;
% FD = 0.5 * rho_air * S * CD.* Va^2;
% Z  = atan((FL-m*g)./(FD));
% [Z_max,I] = max(Z);
% Beta_OPT = Z(1:I);
% 
% yalmip('clear')   
%     X = sdpvar(1,1);
%     problem = [X>=0, X<=8]; % constraints that have to be satisfied
%     cost_function = - beta_opt (X, Va, m, g, S, max_angle);
%     solution = optimize(problem,cost_function);
%     sol = double(X);
%     X0 = sol;%-0.0234;
%     betaf2 = beta_opt(X0, Va, m , g, S, max_angle);
%     if betaf2 <= 40*pi/180
%         betaf = 40*pi/180;
%     else 
%         betaf = betaf2;
%     end
%     betaf = betaf-8*pi/180;

% 
V  = vmag;
X1 = [];
Z  = [];
for X = 0:0.01:8
    syms beta
%   CD  = (-0.0211*X.^3+0.1873*X.^2+0.1183*X+0.5);
%   CL  = (0.0126*X.^4-0.2004*X.^3+0.7482*X.^2+1.3447*X);
    CL = (1 - 1./(1 + exp(-0.904 * X))).*(0.0276*X.^4+4.6595*X.^3-6.0369*X.^2+2.9557*X) +  1./(1 + exp(-0.904 * X)) .*(1.0849*X);
    CD = -0.0461 * X.^3 + 0.5167 * X.^2 -0.6511 * X +0.9081;
    FL = 0.5 * rho_air * S * CL * V^2;
    FD = 0.5 * rho_air * S * CD * V^2;
    eq1 = beta - atan((FL-m*g - T0*sin(beta))./(FD - T0*cos(beta)));
    %eq1 = beta - atan((FL-m*g)./(FD));
    sol = vpasolve(eq1 == 0, beta); 
    Z = [Z, double(sol)];
    X1 = [X1, X];
end
 
[Z_max,I] = max(Z);
Beta_OPT = Z(1:I);
X1 = X1(X1 <= X1(I));
indeces = find(Beta_OPT >= betai);
X1 = X1(indeces);
Beta_OPT = Beta_OPT(indeces);
betaf = Z_max-3*pi/180;
%X0 = X1(1);

X = 0:0.01:8;
if Z_max <= 23*pi/180
    betaf = 45*pi/180;% +betai;
    X1 = [];
    for beta = betai : 0.1*pi/180 : betaf
        CL = (1 - 1./(1 + exp(-0.904 * X))).*(0.0276*X.^4+4.6595*X.^3-6.0369*X.^2+2.9557*X) +  1./(1 + exp(-0.904 * X)) .*(1.0849*X);
        CD = -0.0461 * X.^3 + 0.5167 * X.^2 -0.6511 * X +0.9081;
        fun = CL.*cos(beta) - CD.*sin(beta);
        [Val, I] = max(fun);
        X1 = [X1, X(I)];
    end
    Beta_OPT = betai : 0.1*pi/180 : betaf;
    X0 = 4;
   display('less than 20') 
elseif Z_max > 23*pi/180 && Z_max < 43*pi/180
    betaf = 20*pi/180 +betai;
    Beta = Beta_OPT(Beta_OPT <= Z_max);
    for beta = Z_max: 0.1*pi/180: betaf
      CL = (1 - 1./(1 + exp(-0.904 * X))).*(0.0276*X.^4+4.6595*X.^3-6.0369*X.^2+2.9557*X) +  1./(1 + exp(-0.904 * X)) .*(1.0849*X);
      CD = -0.0461 * X.^3 + 0.5167 * X.^2 -0.6511 * X +0.9081;
      fun = CL.*cos(beta) - CD.*sin(beta);
      [Val,I] = max(fun);
      X1 = [X1, X1(end)];
    end
    Beta_OPT = [Beta, Z_max: 0.1*pi/180: betaf];
    display('less than 40') 
    
end   



%%% ------------- LQR LINEAR CONTROL OF THE SYSTEM  ----------------
    % Double Derivative State Space Representation
    B_DI = [0 0 0
            0 0 0
            0 0 0
            1 0 0
            0 1 0
            0 0 1 ] ;

    A_DI = [0 0 0 1 0 0 ;
            0 0 0 0 1 0 ;
            0 0 0 0 0 1 ;
            0 0 0 0 0 0 ;
            0 0 0 0 0 0 ;
            0 0 0 0 0 0 ] ;

    C_DI = [1 0 0 0 0 0 ;
             0 1 0 0 0 0 ;
             0 0 1 0 0 0 ] ;

    A_DI_extended = [A_DI zeros(6,3) ;
                    -C_DI zeros(3)] ;
    B_DI_extended = [B_DI ; zeros(3) ] ;

    % LQR Weight Matrices
    Q_DI = diag([7*[1 1 1], 5*[1 1 1], 8*[1 1 1]]) ;
    % R_DI = 0.2*eye(3) ;
    R_DI = [1 0 0;
            0 10 0; 
             0  0 20] ;

    % LQR Control Gain
    K_DI_int = lqrd(A_DI_extended, B_DI_extended, Q_DI, R_DI, Ts) ;
    
    
    
    A_LQR =[0 1 0;
             0 0 0;
              1 0 0]; 
    B_LQR = [0 1 0]';
    Q_r = diag([1, 1, 2]); R_r = 0.5;
    Q_etha = diag([0.5, 0.5, 0.5]); R_etha = 0.5;
    Q_beta = diag([0.5, 0.1, 0.4]); R_beta = 0.1;
    k_r = lqr(A_LQR, B_LQR, Q_r, R_r);
    k_etha = lqr(A_LQR, B_LQR, Q_etha, R_etha);
    k_beta = lqr(A_LQR, B_LQR, Q_beta, R_beta);
    K_DI_int = [diag([k_r(1), k_etha(1), k_beta(1)]), diag([k_r(2), k_etha(2), k_beta(2)]), diag([k_r(3), k_etha(3), k_beta(3)])];


% Initial condition of Extended Kalman Filter
vitesse_sys = [0; 0; 0];
quad_int = initial_attitude_quaternion_condition';
input_int = [vitesse_vent(:,1); vitesse_sys; quad_int]; 
Va_int = App_wind(input_int);
data = [CDy; Slat; S; RM];
wmag_int = 0; %vx_wind*3/RM;
[FaLB_int,FaG_int] = Aero_initial(Va_int, data, wmag_int, quad_int);

Rmat = [cos(ethai)*cos(betai) sin(ethai)*cos(betai) sin(betai);
             -sin(ethai)           cos(ethai)          0;
       -cos(ethai)*sin(betai) -sin(ethai)*sin(betai) cos(betai)];

data_d = [Cx zeros(1,2); S_apparente];
[FaLB_d_int, FaG_int_d] = Aero_d_initial(Va_int, data_d, quad_int);

Fa_L_int = Rmat*(FaG_int + FaG_int_d);


%%% -------------------- Generation d'Echelons rampes ---------------------
pente_echelon = 1 ;
temps_montee = 1 ;
temps_plateau = 10 ;
temps_descente = 1 ;
delai = 10 ;

%%% ------------------------- Generation d'Echelons -----------------------
hauteur_echelon = 1.0 ;

% Time parametres 
t_TAKE_OFF = 5 ;
t_x_1 = 20 ;
t_x_2 = t_x_1 + temps_montee ;
t_x_3 = t_x_2 + temps_plateau ;
t_x_4 = t_x_3 + temps_descente ;

t_y_1 = t_x_4 + delai ;
t_y_2 = t_y_1 + temps_montee ;
t_y_3 = t_y_2 + temps_plateau ;
t_y_4 = t_y_3 + temps_descente ;

t_z_1 = t_y_4 + delai ;
t_z_2 = t_z_1 + temps_montee ;
t_z_3 = t_z_2 + temps_plateau ;
t_z_4 = t_z_3 + temps_descente ;

t_xyz_1 = t_z_4 + 2*delai ;
t_xyz_2 = t_xyz_1 + temps_montee ;
t_xyz_3 = t_xyz_2 + temps_plateau ;
t_xyz_4 = t_xyz_3 + temps_descente ;

t_LAND = t_xyz_4 + delai ;

tf_echelons = t_LAND + 2*delai ;
tf_echelons=130;



%% III. CONTROL LAW SYNTHESIS
%% III.0 ROS/PX4 PQR loop  
% ------------------------- ROS/PX4 PQR gains -----------------------------

P_ROLL = 0.1500 ;
I_ROLL = 0.2;
D_ROLL = 0.0022;

P_PITCH = 0.150	 ;
I_PITCH = 0.2;
D_PITCH = 0.0022;


P_YAW = 0.20 ;
I_YAW = 0.10 ; 
D_YAW = 0.0 ;


K_PQR_PX4 = 1*diag([0.8 1.35 1]) ;
P_PQR_PX4 = diag([P_ROLL P_PITCH P_YAW]);
I_PQR_PX4 = diag([I_ROLL I_PITCH I_YAW]) ;
D_PQR_PX4 = diag([D_ROLL D_PITCH D_YAW]) ;


P_ATTITUDE_PX4 = diag([10 10 2.8]) ;
PX4_rate_loop_frequency = 500 ;           % Fréquence boucle rates [Hz]   
PX4_attitude_loop_frequency = 250 ;       % Fréquence boucle attitude [Hz]   

%% 

t1T = t1/10;
t2T = t2 + 9*t1/10;
Ti = 0;
Tf = 1.5;
%temps de réponse
tr=0.8;

%tf_echelons=100;

% X %
tfx_start=20;
tfx_stop=30;
tfx_value=2;

% Y %
tfy_start=40;
tfy_stop=50;
tfy_value=2;

% Z %
tfz_start=10;
tfz_stop=60;
tfz_value=1;

% X, Y, Z %
tfx2_start=60;
tfx2_stop=70;
tfx2_value=2;
tfy2_start=60;
tfy2_stop=70;
tfy2_value=2;
tfz2_start=60;
tfz2_stop=70;
tfz2_value=2;
% Yaw %
tfyaw_start=80;
tfyaw_stop=90;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Trajectory %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% -------------------- Generation d'Echelons rampes ---------------------
pente_echelon = 1 ;
temps_montee = 1 ;
temps_plateau = 10 ;
temps_descente = 1 ;
delai = 10 ;

%%% ------------------------- Generation d'Echelons -----------------------
hauteur_echelon = 1.0 ;

% Time parametres 
t_TAKE_OFF = 5 ;
tf_echelons=130;

%%% ------------------------- LQR Control -----------------------
A = [0 1 0;
      0 0 0;
      -1 0 0]; 
B = [0 1 0]';

Q_beta = diag([1.2, 0.6, 1]);
R_beta = 0.4;
L = lqr(A,B,Q_beta,R_beta);


%% Klaman Initialization

vitesse_sys=[0;0;0];
quad_int=initial_attitude_quaternion_condition';
input_int=[vitesse_vent; vitesse_sys;quad_int]; 
Va_int=App_wind(input_int);
data=[CDy; Slat; S; RM];
wmag_int=0;%vx_wind*3/RM;
[FaLB_int,FaG_int]= Aero_initial(Va_int,data, wmag_int, quad_int);
Rmat=[cos(ethai)*cos(betai) sin(ethai)*cos(betai) sin(betai);
-sin(ethai) cos(ethai) 0;
-cos(ethai)*sin(betai) -sin(ethai)*sin(betai) cos(betai)];

data_d=[Cx zeros(1,2);S_apparente];
[FaLB_d_int,FaG_int_d]=Aero_d_initial(Va_int,data_d,quad_int);

Fa_L_int=Rmat*(FaG_int+FaG_int_d);


%%%  ------------------------- Functions -----------------------
function [Z, FL, FD] = beta_opt(X, V, m, g, S,max_angle)
CL = (1 - 1./(1 + exp(-0.904 * X))).*(0.0276*X.^4+4.6595*X.^3-6.0369*X.^2+2.9557*X) +  1./(1 + exp(-0.904 * X)) .*(1.0849*X);
CD = -0.0461 * X.^3 + 0.5167 * X.^2 -0.6511 * X +0.9081;
FL = 0.5*S*CL*V^2;
FD = 0.5*S*CD*V^2;
Z = atan((FL-m*g)/(FD));
end
