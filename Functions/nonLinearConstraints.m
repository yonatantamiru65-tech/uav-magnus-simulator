%{
   This function calculates the nonlinear constraines for the Control Allocation optimization.

   Input:
      X1 - The input vector [X, theta]
      Va - Apparent wind Velocity vector in the yaw frame
      data - Data vector containing parameters
      CForces - Desired Forces vector in the yaw frame

   Output:
      c - empty Inequality constraints
      ceq - constraints on the Euler angles, thrust and spin ratio

   Author: Zakeye AZAKI 
   Date: July, 2023
%}
function [c, ceq] = nonLinearConstraints(X, Va_B, data, quaternion, pos_spherical, F_control_spher, X_prev, r_ddot, u_r, psi)

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
    m_w = data(9);
    
    psi_vicon = 0;
    
% Unpack component pos_spherical
    etha  = pos_spherical(2);
    beta = pos_spherical(3);
    
    % Unpacj component quaternion
    q0 = quaternion(1);
    q1 = quaternion(2); 
    q2 = quaternion(3);
    q3 = quaternion(4);
    

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
    
    R_GS = [cos(etha)*cos(beta) sin(etha)*cos(beta) sin(beta);
                -sin(etha) cos(etha) 0;
                 -cos(etha)*sin(beta) -sin(etha)*sin(beta) cos(beta)];

    % Calculate residual forces
    F_aero_spher =  R_GS * R_LG * Fa_LB';
    F_res = F_control_spher - R_GS * R_LG * Fa_LB';

    
    % Extract components of residual forces
    u_T    = F_res(1);
    u_etha = F_res(3);
    u_beta = F_res(3);
    
%     if Vaxz >3.5
%         Z=1;
%     else 
%         Z=0;
%     end 
%     
    % Calculate the thrust J (constraint)
    J = max(u_T,0)^2 + u_etha^2 + u_beta^2;
    theta = atan2((u_T*cos(etha)*cos(beta) - u_etha*sin(etha) - u_beta*cos(etha)*sin(beta)) * cos(psi+psi_vicon) +...
        (u_T*sin(etha)*cos(beta) + u_etha*cos(etha) - u_beta*sin(etha)*sin(beta)) * sin(psi+psi_vicon), (u_T*sin(beta) + u_beta*cos(beta)));

    % Define inequality constraints
    c = [theta - max_Eulerangle, -theta - max_Eulerangle, sqrt(J) - 2*m*g (X * Vaxz / RM) * 60 / (2 * pi) - 24000];
    %, abs(X-X_prev) - 2.16, 0-(m_w*r_ddot +u_r +0.4)];...
        %Z*(-(F_res(1)+F_control_spher(1))+(F_control_spher(1)+1))];
end