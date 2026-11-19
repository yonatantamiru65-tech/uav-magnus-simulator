    global CDy Slat S RM max_Eulerangle m g rho_air CForces
    max_Eulerangle = 35*pi/180;
    CForces=[0 0 m*g];

    % Define the range of values for X, theta, and V
    X_range = linspace(0, 8, 20);
    theta_range = linspace(0, 35*pi/180, 20);
    V_range = -[4 7 9];

    % Create a meshgrid for X, theta, and V
    [X, V, Theta] = meshgrid(X_range, V_range, theta_range);

    % Reshape X, Theta, and V to 1D vectors
    X_1d = reshape(X, [], 1);
    Theta_1d = reshape(Theta, [], 1);
    V_1d = reshape(V, [], 1);

    % Compute the cost function for each point in the 1D vectors
    J_1d = [];
    for i=1:1:length(X_1d)
    J_1d = [J_1d costfunction(X_1d(i), V_1d(i), Theta_1d(i))];
    end

    % Reshape the cost function results to 3D array
    J = reshape(J_1d, size(X));

    % Create a 3D contour plot using isosurface
    figure;
    slice(X, -V, Theta*180/pi, J, [], [4 7 9], []); % You can adjust the isovalue

    % Add labels and title
    customTicks = [4, 7, 9]; % for 7m/s data 
    yticks(customTicks);
    xlabel('Spin ratio [-]', 'Interpreter', 'latex');
    ylabel('Linear velocity [m.s$^{-1}$]', 'Interpreter', 'latex');
    zlabel('Pitch [deg]', 'Interpreter', 'latex');
    title('3D Contour Plot of Cost Function', 'Interpreter', 'latex');
    digitsFont = 23;
    set(gca,'Fontsize',digitsFont)

    % Add color to the contours
    colormap('jet');
    colorbar;
    %print('Contour_Plot_Cost_Function','-depsc2')
function J = costfunction(X, V, theta)
global CDy Slat S RM max_Eulerangle m g rho_air CForces
%V = -9.3;
    % Calculate rotation matrix
    R_Pitch = [cos(theta), 0, sin(theta);
         0, 1, 0;
         -sin(theta), 0, cos(theta)];
    
    % Transform Va to local frame
    Va = [V 0 0];
    Va_Pitch = R_Pitch \ Va';

    % Calculate lateral force due to Magnus effect
    Vaxz = norm([Va_Pitch(1), Va_Pitch(3)]);
    % Calculate normalized drag direction vector
    if Vaxz == 0
         eFD = [1, 0, 0];
    else
         eFD = [Va_Pitch(1) / Vaxz, 0, Va_Pitch(3) / Vaxz];
    end

    % Calculate normalized lift direction vector
    eFL = cross([0, 1, 0], eFD);
    
    % Calculate CD and CL based on X
    CL = (1 - 1./(1 + exp(-0.904 * X))).*(0.0276*X^4+4.6595*X^3-6.0369*X^2+2.9557*X) +  1./(1 + exp(-0.904 * X)) .*(1.0849*X);
    CD = -0.0461 * X^3 + 0.5167 * X^2 -0.6511 * X +0.9081;

    % Calculate lift and drag forces in pitch local frame
    F_L_xz_local = (1/2) * rho_air* CL * S * Vaxz^2 * eFL;
    F_D_xz_local = (1/2) * rho_air* CD * S * Vaxz^2 * eFD;
    Fa_LB = F_D_xz_local + F_L_xz_local;

    % Calculate residual forces
    F_res = R_Pitch \ CForces' -  Fa_LB';

    % Extract components of residual forces
    Fx_res = F_res(1);
    Fy_res = F_res(2);
    Fz_res = F_res(3);


    % Calculate the cost function
    J = sqrt((Fx_res^2 + Fz_res^2 + Fy_res^2))+ 3 * sqrt((theta - atan2(Fx_res, Fz_res))^2) ;
end
