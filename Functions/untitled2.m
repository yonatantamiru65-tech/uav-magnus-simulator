%{
   This function calculates the cost for optimization.

   Input:
      X1 - The input vector [X, theta]
      Va - Apparent wind Velocity vector in the yaw frame
      data - Data vector containing parameters
      CForces - Desired Forces vector in the yaw frame

   Output:
      J - The calculated cost

   Author: Zakeye AZAKI 
   Date: July, 2023
%}
function J = costFunction2(X1, Va, data, CForces)

    % Unpack data
    CDy = data(1);
    Slat = data(2);
    S = data(3);
    RM = data(4);
    max_Eulerangle = data(5);
    m = data(6);
    g = data(7);
    rho_air = data(8);
    
    % Extract variables from X1
    X = X1;

    % Calculate rotation matrix
    R_Pitch = [cos(theta), 0, sin(theta);
         0, 1, 0;
         -sin(theta), 0, cos(theta)];

    % Transform Va to local frame
    Va_Pitch = R_Pitch .\ Va';

    % Calculate lateral force due to Magnus effect
    F_a_y_local = (1/2) * rho_air* CDy * Slat * abs(Va_Pitch(2)) * Va_Pitch(2);
    Vaxz = norm([Va_Pitch(1), Va_Pitch(3)]);
    Vaxy = norm([Va(1), Va(2)]);
    % Calculate normalized drag direction vector
    if Vaxz == 0
         eFD = [1, 0, 0];
    else
         eFD = [Va_Pitch(1) / Vaxz, 0, Va_Pitch(3) / Vaxz];
    end

    % Calculate normalized lift direction vector
    eFL = cross([0, 1, 0], eFD);
    
    % Calculate CD and CL based on X
    % BED MODEL
    %CL = 0.0725 * X^4 -0.9854 * X^3 + 3.9570 * X^2 -2.2335 * X +0.1402;
    %CD = -0.0740 * X^3 + 0.7597 * X^2 -1.1766 * X + 1.0932; 
    CL = (1 - 1./(1 + exp(-0.904 * X))).*(0.0276*X^4+4.6595*X^3-6.0369*X^2+2.9557*X) +  1./(1 + exp(-0.904 * X)) .*(1.0849*X);
    CD = -0.0461 * X^3 + 0.5167 * X^2 -0.6511 * X +0.9081;
    
    % YASH MODEL 
    %CD = (-0.0211 * X^3 + 0.1873 * X^2 + 0.1183 * X + 0.5);
    %CL = (0.0126 * X^4 - 0.2004 * X^3 + 0.7482 * X^2 + 1.3447 * X);

    % Calculate lift and drag forces in pitch local frame
    F_L_xz_local = (1/2) * rho_air* CL * S * Vaxz^2 * eFL;
    F_D_xz_local = (1/2) * rho_air* CD * S * Vaxz^2 * eFD;
    Fa_LB = [0, F_a_y_local, 0] + F_D_xz_local + F_L_xz_local;

    % Calculate residual forces
    F_res = CForces' - R_Pitch * Fa_LB';

    % Extract components of residual forces
    Fx_res = F_res(1);
    Fy_res = F_res(2);
    Fz_res = F_res(3);
    

    % Calculate the cost function
    J = 4*(Fx_res^2 + Fz_res^2 + Fy_res^2) + 3 *(theta - atan2(Fx_res, Fz_res))^2;
end