function [optimized_X, F_res_spherical, Fa_zLB, fval, exitflag, gradient] = runOptimizationHelper(data)
% Full MATLAB-side implementation of optimization and aerodynamic forces

% ---------------- Unpack input data ----------------
pos_spherical = data(1:3); 
F_control_spher = data(4:6);
Va_G = data(7:9);
comp = data(10:18);
X0 = data(19);
quaternion = data(20:23);
psi = data(24);
r_ddot = data(25);
u_r = data(26);

% Component parameters
CDy = comp(1); 
Slat = comp(2);
S = comp(3);
RM = comp(4);
max_Eulerangle = comp(5);
m = comp(6);
g = comp(7);
rho_air = comp(8);
m_w = comp(9);

persistent X_prev
if isempty(X_prev)
    X_prev = 0;
end

% Quaternion rotation to body frame
q0 = quaternion(1); q1 = quaternion(2); q2 = quaternion(3); q3 = quaternion(4);
R_matrix = [ ...
    (q0^2 + q1^2 - q2^2 - q3^2),  2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
    2*(q1*q2 + q0*q3),  (q0^2 - q1^2 + q2^2 - q3^2), 2*(q2*q3 - q0*q1);
    2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0^2 - q1^2 - q2^2 + q3^2)];
Va_B = R_matrix \ Va_G;

% ---------------- Optimization ----------------
options = optimoptions('fmincon','Algorithm','sqp');
cost  = @(X) costFunction(X, Va_B, comp, quaternion, pos_spherical, F_control_spher);
noncon = @(X) nonLinearConstraints(X, Va_B, comp, quaternion, pos_spherical, ...
                                   F_control_spher, X_prev, r_ddot, u_r, psi);

[x_opt,fval,exitflag,~,~,gradient] = ...
    fmincon(cost, X0, [], [], [], [], 0, 8, noncon, options);
X_prev = x_opt;

% ---------------- Aerodynamic Calculations ----------------
F_a_y_local = 0.5 * rho_air * CDy * Slat * abs(Va_B(2)) * Va_B(2);
Vaxz = norm([Va_B(1), Va_B(3)]);

if Vaxz == 0
    eFD = [1, 0, 0];
else
    eFD = [Va_B(1)/Vaxz, 0, Va_B(3)/Vaxz];
end
eFL = cross([0, 1, 0], eFD);

% Lift and drag coefficients
CL = (1 - 1./(1 + exp(-0.904 * x_opt))) .* ...
     (0.0276*x_opt^4 + 4.6595*x_opt^3 - 6.0369*x_opt^2 + 2.9557*x_opt) + ...
     1./(1 + exp(-0.904 * x_opt)) .* (1.0849*x_opt);
CD = -0.0461*x_opt^3 + 0.5167*x_opt^2 - 0.6511*x_opt + 0.9081;

% Lift and drag forces
F_L_xz_local = 0.5 * rho_air * CL * S * Vaxz^2 * eFL;
F_D_xz_local = 0.5 * rho_air * CD * S * Vaxz^2 * eFD;
Fa_LB = [0, F_a_y_local, 0] + F_D_xz_local + F_L_xz_local;

% Coordinate transformations
r = pos_spherical(1);
eta = pos_spherical(2);
beta = pos_spherical(3);
R_LG  = [ ...
    (q0^2 + q1^2 - q2^2 - q3^2),  2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
    2*(q1*q2 + q0*q3),  (q0^2 - q1^2 + q2^2 - q3^2), 2*(q2*q3 - q0*q1);
    2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0^2 - q1^2 - q2^2 + q3^2)];
R_GS = [ ...
    cos(eta)*cos(beta), sin(eta)*cos(beta), sin(beta);
    -sin(eta),          cos(eta),           0;
    -cos(eta)*sin(beta), -sin(eta)*sin(beta), cos(beta)];

% Residual and final forces
F_res = F_control_spher - R_GS * R_LG * Fa_LB';
F_res_spherical = [max(F_res(1),0.53); F_res(2); F_res(3)];
Fa_zLB = Fa_LB(3);

% ---------------- Type Enforcement for Simulink ----------------
optimized_X = double(x_opt);
fval = double(fval);
gradient = double(gradient);
exitflag = int8(exitflag);  % fixed type for Simulink
F_res_spherical = double(F_res_spherical);
Fa_zLB = double(Fa_zLB);
end
