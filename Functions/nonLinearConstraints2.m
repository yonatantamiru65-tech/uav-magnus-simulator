function [c, ceq] = nonLinearConstraints2(X, Va_B, data, pos_spherical, F_control_spher, R)

    % Initialize empty inequality constraint array (no inequality constraints in this case)
    c = [];
    
    % Initialize empty equality constraint array
    ceq = [];
    
    % Unpack data
    CDy = data(1); 
    Slat = data(2);
    S = data(3);
    RM = data(4);
    max_Eulerangle = data(5);
    m = data(6);
    g = data(7);
    rho_air = data(8);
    
% Unpack component pos_spherical
    eta  = pos_spherical(2);
    beta = pos_spherical(3);
    
    % Calculate lateral force due to Magnus effect
    F_a_y_local = (1/2) * rho_air* CDy * Slat * abs(Va_B(2)) * Va_B(2);
    Vaxz = norm([Va_B(1), Va_B(3)]);
    % Calculate normalized drag direction vector
    if Vaxz == 0
         eFD = [1, 0, 0];
    else
         eFD = [Va_B(1) / Vaxz, 0, Va_B(3) / Vaxz];
    end

    % Calculate normalized lift direction vector
    eFL = cross([0, 1, 0], eFD);
    
    % Calculate CD and CL based on X
    % BED MODEL
    CL = (1 - 1./(1 + exp(-0.904 * X))).*(0.0276*X^4+4.6595*X^3-6.0369*X^2+2.9557*X) +  1./(1 + exp(-0.904 * X)) .*(1.0849*X);
    CD = -0.0461 * X^3 + 0.5167 * X^2 -0.6511 * X +0.9081;
    
    % Calculate lift and drag forces in pitch local frame
    F_L_xz_local = (1/2) * rho_air* CL * S * Vaxz^2 * eFL;
    F_D_xz_local = (1/2) * rho_air* CD * S * Vaxz^2 * eFD;
    Fa_LB = [0, F_a_y_local, 0] + F_D_xz_local + F_L_xz_local;
    
    % Calculate residual forces
    F_res = F_control_spher - R * Fa_LB';

    
    % Extract components of residual forces
    u_T    = F_res(1);
    u_etha = F_res(3);
    u_beta = F_res(3);
    
    % Calculate the thrust J (constraint)
    J = max(u_T,0)^2 + u_etha^2 + u_beta^2;
    theta = atan2((u_T*cos(eta)*cos(beta) - u_etha*sin(eta) - u_beta*cos(eta)*sin(beta)) / cos(pi) , (u_T*sin(beta) + u_beta*cos(beta)));

    % Define inequality constraints
    c = [theta - max_Eulerangle, -theta - max_Eulerangle, sqrt(J) - 2*m*g, (X * Vaxz / RM) * 60 / (2 * pi) - 24000];
end
