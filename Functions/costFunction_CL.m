function J = costFunction_CL(optimal_sol, Va_B, data, quaternion, pos_spherical, F_control_spher)

    
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
    r    = pos_spherical(1);
    eta  = pos_spherical(2);
    beta = pos_spherical(3);
    
    % Unpacj component quaternion
    q0 = quaternion(1);
    q1 = quaternion(2); 
    q2 = quaternion(3);
    q3 = quaternion(4);
    
    X = optimal_sol(1);
    u_r = optimal_sol(2); 
    uT = optimal_sol(3);
    
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
    R_LG = [(q0^2 + q1^2 - q2^2 - q3^2)  2*(q1*q2 - q0*q3) 2*(q1*q3 + q0*q2) ;
                 2*(q1*q2 + q0*q3)  (q0^2 - q1^2 + q2^2 - q3^2) 2*(q2*q3 - q0*q1) ;
                 2*(q1*q3 - q0*q2) 2*(q2*q3 + q0*q1) (q0^2 - q1^2 - q2^2 + q3^2) ] ;
    
    R_GS = [cos(eta)*cos(beta) sin(eta)*cos(beta) sin(beta);
                -sin(eta) cos(eta) 0;
                 -cos(eta)*sin(beta) -sin(eta)*sin(beta) cos(beta)];

    % Calculate residual forces
    F_res = [uT, F_control_spher(2), F_control_spher(3)]' - R_GS * R_LG * Fa_LB';

    % Extract components of residual forces
    FT_res    = F_res(1);
    Feta_res  = F_res(2);
    Fbeta_res = F_res(3);

    J = ((max(FT_res,0))^2 + Feta_res^2 + Fbeta_res^2) + 5 * abs(F_control_spher(1) -m*g*sin(beta) -u_r);
end
