
function J = costFunction2(X, Va_B, data, F_control_spher, R)

    
    % Unpack data
    CDy = data(1);
    Slat = data(2);
    S = data(3);
    rho_air = data(8);
   
    % Calculate lateral force due to Magnus effect
    F_a_y_local = (1/2) * rho_air* CDy * Slat * abs(Va_B(2)) * Va_B(2);
    Vaxz = norm([Va_B(1), Va_B(3)]);
    Vaxy = norm([Va_B(1), Va_B(2)]);
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
    FT_res    = F_res(1);
    Feta_res  = F_res(2);
    Fbeta_res = F_res(3);
    
    if Vaxy < 1.5 
        Z = 1 ;
    else
        Z = 0 ;
    end

    % Calculate the cost function
    J = (max(FT_res,0))^2 + Feta_res^2 + Fbeta_res^2;
end
