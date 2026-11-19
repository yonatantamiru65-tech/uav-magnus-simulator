theta =  pi/2 - 54.89*pi/180;
eta = 0;
beta = 40*pi/180; %Z_max; %30*pi/180;% betaf+3*pi/180;

F_res = [1.5 + m*g*sin(beta), 0, m*g*cos(beta)];
FT = F_res(1);
Feta = F_res(2);
Fbeta = F_res(3);

R_LG  = [-cos(theta), 0, -sin(theta);
              0,      -1,     0;
         -sin(theta), 0, cos(theta)];
     
R_GS  = [ cos(eta)*cos(beta) sin(eta)*cos(beta) sin(beta);
           -sin(eta) cos(eta) 0;
            -cos(eta)*sin(beta) -sin(eta)*sin(beta) cos(beta) ]; 
        
X_val = linspace(1,8,100);
V = [3.5, 0, 0]';
Va_B= R_LG * V;
Vaxz = norm([Va_B(1), 0, Va_B(3)]);
eFD = [Va_B(1), 0, Va_B(3)]/ Vaxz;
eFL = cross([0 1 0] , eFD);

    syms X;
    CL = (1 - 1./(1 + exp(-0.904 * X))).*(0.0276*X.^4+4.6595*X.^3-6.0369*X.^2+2.9557*X) +  1./(1 + exp(-0.904 * X)) .*(1.0849*X);
    CD = -0.0461 * X.^3 + 0.5167 * X.^2 -0.6511 * X +0.9081;
    
    F_L = (1/2) * rho_air* CL * S * Vaxz^2 * eFL;
    F_D = (1/2) * rho_air* CD * S * Vaxz^2 * eFD;
    
    F_aero = R_GS * inv(R_LG) * (F_L + F_D)';
    JT = (FT - F_aero(1))^2;
    Jeta = (Feta - F_aero(2))^2;
    Jbeta = (Fbeta - F_aero(3))^2;
    
    J_max = (FT - F_aero(1))^2 + Jeta + Jbeta;
    J_min = 0.53^2 + Jeta + Jbeta;
    dJ_max = diff(J_max, X);
    dJ_min = diff(J_min, X);
   
    % Convert symbolic expression to a MATLAB function
    f_J_max = matlabFunction(J_max);
    f_J_min = matlabFunction(J_min);
    f_dJ_max = matlabFunction(dJ_max);
    f_dJ_min = matlabFunction(dJ_min);
    f_Jbeta = matlabFunction(Jbeta);
    f_dJbeta = matlabFunction(diff(Jbeta, X));
    f_JT = matlabFunction(JT);
    f_dJT = matlabFunction(diff(JT, X));
    
JGrad = []; J = []; Jbeta_diff = []; Jbeta = []; JT = []; JT_diff = [];
for i = 1:length(X_val)
    X = X_val(i);
    CL = (1 - 1./(1 + exp(-0.904 * X))).*(0.0276*X^4+4.6595*X^3-6.0369*X^2+2.9557*X) +  1./(1 + exp(-0.904 * X)) .*(1.0849*X);
    CD = -0.0461 * X^3 + 0.5167 * X^2 -0.6511 * X +0.9081;
    
    F_L = (1/2) * rho_air* CL * S * Vaxz^2 * eFL;
    F_D = (1/2) * rho_air* CD * S * Vaxz^2 * eFD;
    
    F_aero = R_GS * R_LG * (F_L + F_D)';
    
    if  FT - F_aero(1) > 0.53
      J = [J, f_J_max(X)];
      JGrad = [JGrad, f_dJ_max(X)];
      JT = [JT, f_JT(X)];
      JT_diff = [JT_diff, f_dJT(X)];
    else 
      J = [J, f_J_min(X)];
      JGrad = [JGrad, f_dJ_min(X)];
      JT = [JT, 0.53^2];
      JT_diff = [JT_diff, 0];
    end
    Jbeta = [Jbeta, f_Jbeta(X)];
    Jbeta_diff = [Jbeta_diff, f_dJbeta(X)];
end

figure;
plot(X_val, JGrad)
hold on; 
plot(X_val, J)
hold on; 
yline(0);
title('JGrad');

figure;
plot(X_val, JT_diff)
hold on; 
plot(X_val, JT)
hold on; 
yline(0);
title('JT')

figure()
plot(X_val, Jbeta_diff)
hold on; 
plot(X_val, Jbeta)
hold on; 
yline(0);
title('Jbeta_diff')