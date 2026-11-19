windowSize = 30;

%% STSMC Loading Data
load("Data_3ms_4000rpm_STSMC_SIM_3D.mat")%4ms 4000 rpm
Ts_STSM = 0.002;
Data = Simulation_Data.signals.values;
index_STSM = 1 : 1: 45/Ts_STSM ;


Pos_ref = Data(index_STSM,1:3) ;
Speed_ref = Data(index_STSM,4:6);
pqr_ref = Data(index_STSM,7:9) ;
Euler123_ref = Data(index_STSM,10:12) ;

state_feedback = [Data(index_STSM,13:18), Data(index_STSM,22:24), Data(index_STSM,19:21)] ;

motor_speeds_STSM = Data(index_STSM, 25:28);
Twinch_PL_STSM = smoothdata(Data(index_STSM, 29), 'movmean', 50);
Log_tension_ref_STSM = Data(index_STSM, 30);
Tension_Magnitude_STSM = Data(index_STSM, 31); 
Friction_Force_STSM =  -Data(index_STSM, 32);
Tension_Force_STSM = Data(index_STSM, 35:37); 
Control_Torques_STSM = Data(index_STSM, 38:40); 
S_SMC_STSM= Data(index_STSM, 41:43);
Total_additional_forces_STSM = Data(index_STSM, 44:46);
Estimated_additional_forces_STSM = Data(index_STSM, 47:49);
Estimated_pos_STSM = Data(index_STSM, 50:52);
Estimated_speed_STSM = Data(index_STSM, 53:55);
wind_STSM = Data(index_STSM, 56:58);
Thrust_ref_N_STSM = Data(index_STSM, 59);

r_ref_STSM = Pos_ref(:,1) ;
etha_ref_STSM = Pos_ref(:,2)*180/pi ;
beta_ref_STSM = Pos_ref(:,3)*180/pi ;
r_dot_ref_STSM = Speed_ref(:,1) ;
etha_dot_ref_STSM = Speed_ref(:,2)*180/pi ;
beta_dot_ref_STSM = Speed_ref(:,3)*180/pi ;

sf_rl_STSM = state_feedback(:,1);
sf_etha_STSM = state_feedback(:,2)*180/pi;
sf_beta_STSM = state_feedback(:,3)*180/pi;
sf_rdot_STSM = state_feedback(:,4) ;
sf_ethadot_STSM = state_feedback(:,5)*180/pi ;
sf_betadot_STSM = state_feedback(:,6)*180/pi;

sf_phi_STSM = state_feedback(:,7)* 180/pi;
sf_theta_STSM = state_feedback(:,8)*180/pi;
sf_psi_STSM = state_feedback(:,9)*180/pi;

% Number of data points
nd = length(sf_rl_STSM); 

% Initialize variables
mu_r = 0; mu_eta = 0; mu_beta = 0;
sum_mu_r = 0; sum_mu_eta = 0; sum_mu_beta = 0;
sum_sigma_r = 0; sum_sigma_eta = 0; sum_sigma_beta = 0;
sum_var_r = 0; sum_var_eta = 0; sum_var_beta = 0;

% Compute mu (mean absolute error)
for k = 1:nd
    sum_mu_r = sum_mu_r + norm(sf_rl_STSM(k,:) - r_ref_STSM(k,:));
    sum_mu_eta = sum_mu_eta + norm(sf_etha_STSM(k,:) - etha_ref_STSM(k,:));
    sum_mu_beta = sum_mu_beta + norm(sf_beta_STSM(k,:) - beta_ref_STSM(k,:));
end
mu_r = sum_mu_r / nd;
mu_eta = sum_mu_eta / nd;
mu_beta = sum_mu_beta / nd;

% Compute sigma (standard deviation)
for k = 1:nd
    sum_sigma_r = sum_sigma_r + (norm(sf_rl_STSM(k,:) - r_ref_STSM(k,:)) - mu_r)^2;
    sum_sigma_eta = sum_sigma_eta + (norm(sf_etha_STSM(k,:) - etha_ref_STSM(k,:)) - mu_eta)^2;
    sum_sigma_beta = sum_sigma_beta + (norm(sf_beta_STSM(k,:) - beta_ref_STSM(k,:)) - mu_beta)^2;
end
sigma_r = sqrt(sum_sigma_r / (nd - 1));
sigma_eta = sqrt(sum_sigma_eta / (nd - 1));
sigma_beta = sqrt(sum_sigma_beta / (nd - 1));

% Compute variance (of derivatives)
mean_r_dot = mean(sf_rdot_STSM);  % Mean of xi_dot over all time steps
mean_eta_dot = mean(sf_ethadot_STSM); 
mean_beta_dot = mean(sf_betadot_STSM); 
for k = 1:nd
    sum_var_r = sum_var_r + norm(sf_rdot_STSM(k,:) - mean_r_dot)^2;
    sum_var_eta = sum_var_eta + norm(sf_ethadot_STSM(k,:) - mean_eta_dot)^2;
    sum_var_beta = sum_var_beta + norm(sf_betadot_STSM(k,:) - mean_beta_dot)^2;
end
variance_r = sum_var_r / nd;
variance_eta = sum_var_eta / nd;
variance_beta = sum_var_beta / nd;

% Display results
fprintf('Mean Absolute Error (mu): %.4f\n %.4f\n %.4f\n', mu_r, mu_eta, mu_beta);
fprintf('Standard Deviation (sigma): %.4f\n %.4f\n %.4f\n', sigma_r, sigma_eta, sigma_beta);
fprintf('Variance (var): %.4f\n %.4f\n %.4f\n', variance_r, variance_eta, variance_beta);


%% SMC Loading Data
load("Data_3ms_4000rpm_SMC_SIM_3D_2.mat") %4ms 4000 rpm
Ts_SMC = 0.002;
Data = squeeze(Simulation_Data.signals.values)';
index_SMC = 1 : 1: 45/Ts_SMC ;


Pos_ref = Data(index_SMC,1:3); 
Speed_ref = Data(index_SMC,4:6);
pqr_ref = Data(index_SMC,7:9) ;
Euler123_ref = Data(index_SMC,10:12) ;

state_feedback = [Data(index_SMC,13:18), Data(index_SMC,22:24), Data(index_SMC,19:21)] ;

motor_speeds_SMC = Data(index_SMC, 25:28);
Twinch_PL_SMC = smoothdata(Data(index_SMC, 29), 'movmean', 50);
Log_tension_ref_SMC = Data(index_SMC, 30);
Tension_Magnitude_SMC = Data(index_SMC, 31); 
Friction_Force_SMC =  -Data(index_SMC, 32);
Tension_Force_SMC = Data(index_SMC, 35:37); 
Control_Torques_SMC = Data(index_SMC, 38:40); 
S_SMC_SMC= Data(index_SMC, 41:43);
Thrust_ref_N_SMC = Data(index_SMC, 59);

r_ref_SMC = Pos_ref(:,1) ;
etha_ref_SMC = Pos_ref(:,2)*180/pi ;
beta_ref_SMC = Pos_ref(:,3)*180/pi ;
r_dot_ref_SMC = Speed_ref(:,1) ;
etha_dot_ref_SMC = Speed_ref(:,2) ;
beta_dot_ref_SMC = Speed_ref(:,3) ;

sf_rl_SMC = state_feedback(:,1);
sf_etha_SMC = state_feedback(:,2)*180/pi;
sf_beta_SMC = state_feedback(:,3)*180/pi;
sf_rdot_SMC = state_feedback(:,4) ;
sf_ethadot_SMC = state_feedback(:,5) ;
sf_betadot_SMC = state_feedback(:,6);

sf_phi_SMC = state_feedback(:,7)* 180/pi;
sf_theta_SMC = state_feedback(:,8)*180/pi;
sf_psi_SMC = state_feedback(:,9)*180/pi;

% Number of data points
nd = length(sf_rl_SMC); 

% Initialize variables
mu_r = 0; mu_eta = 0; mu_beta = 0;
sum_mu_r = 0; sum_mu_eta = 0; sum_mu_beta = 0;
sum_sigma_r = 0; sum_sigma_eta = 0; sum_sigma_beta = 0;
sum_var_r = 0; sum_var_eta = 0; sum_var_beta = 0;


for k = 1:nd
    sum_mu_r = sum_mu_r + norm(sf_rl_SMC(k,:) - r_ref_SMC(k,:));
    sum_mu_eta = sum_mu_eta + norm(sf_etha_SMC(k,:) - etha_ref_SMC(k,:));
    sum_mu_beta = sum_mu_beta + norm(sf_beta_SMC(k,:) - beta_ref_SMC(k,:));
end

mu_r = sum_mu_r / nd;
mu_eta = sum_mu_eta / nd;
mu_beta = sum_mu_beta / nd;

% Compute sigma (standard deviation)
for k = 1:nd
    sum_sigma_r = sum_sigma_r + (norm(sf_rl_SMC(k,:) - r_ref_SMC(k,:)) - mu_r)^2;
    sum_sigma_eta = sum_sigma_eta + (norm(sf_etha_SMC(k,:) - etha_ref_SMC(k,:)) - mu_eta)^2;
    sum_sigma_beta = sum_sigma_beta + (norm(sf_beta_SMC(k,:) - beta_ref_SMC(k,:)) - mu_beta)^2;
end
sigma_r = sqrt(sum_sigma_r / (nd - 1));
sigma_eta = sqrt(sum_sigma_eta / (nd - 1));
sigma_beta = sqrt(sum_sigma_beta / (nd - 1));

% Compute variance (of derivatives)
mean_r_dot = mean(sf_rdot_SMC);  % Mean of xi_dot over all time steps
mean_eta_dot = mean(sf_ethadot_SMC); 
mean_beta_dot = mean(sf_betadot_SMC); 
for k = 1:nd
    sum_var_r = sum_var_r + norm(sf_rdot_SMC(k,:) - mean_r_dot)^2;
    sum_var_eta = sum_var_eta + norm(sf_ethadot_SMC(k,:) - mean_eta_dot)^2;
    sum_var_beta = sum_var_beta + norm(sf_betadot_SMC(k,:) - mean_beta_dot)^2;
end
variance_r = sum_var_r / nd;
variance_eta = sum_var_eta / nd;
variance_beta = sum_var_beta / nd;

% Display results
fprintf('Mean Absolute Error (mu): %.4f\n %.4f\n %.4f\n', mu_r, mu_eta, mu_beta);
fprintf('Standard Deviation (sigma): %.4f\n %.4f\n %.4f\n', sigma_r, sigma_eta, sigma_beta);
fprintf('Variance (var): %.4f\n %.4f\n %.4f\n', variance_r, variance_eta, variance_beta);

%%
time_STSM = Ts_STSM * (0: 1: length(index_STSM)-1);
time_SMC  = Ts_SMC  * (0: 1: length(index_SMC) -1);

%% PLot Sizes
digitsFont = 35;
testFont= 25;
LineFont = 1.5;
linewidth = 1.5;

x_points1 = [0, 0, 15, 15];
x_points2 = [30, 30, 45, 45];
x1 = 0; x2 = 45;

%% Trcking Error Plot

fig = figure('Name', 'tracking error r', 'NumberTitle', 'off');

y1 = -0.12; y2 = 0.08;
y_points1 = [y1, y2, y2, y1];
a = fill(x_points1, y_points1, 'b');
a.FaceAlpha = 0.1;
a.EdgeColor = 'none';
uistack(a, 'bottom');

hold on;

y_points2 = y_points1;
a1 = fill(x_points2, y_points2, 'r');
a1.FaceAlpha = 0.1;
a1.EdgeColor = 'none';
uistack(a1, 'bottom');

h1 = plot(time_SMC, r_ref_SMC - sf_rl_SMC, 'b', 'LineWidth', linewidth); hold on; 
h2 = plot(time_STSM, r_ref_STSM - sf_rl_STSM, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', linewidth); hold on; 
grid on;

xlabel('Time [s]');
title('Evolution of Tracking Error in $r$-loop [m]', 'Interpreter', 'latex');
%set(gca, 'FontSize', digitsFont);

% Axis limits
axis([x1 x2 y1 y2]); % 4m/s

exportgraphics(gca,'Tracking_Error_4ms_r_STSM_SMC.pdf','ContentType','vector')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


fig = figure('Name', 'Tracking Error etha', 'NumberTitle', 'off');

y1 = -10; y2 = 7;
y_points1 = [y1, y2, y2, y1];
a = fill(x_points1, y_points1, 'b');
a.FaceAlpha = 0.1;
a.EdgeColor = 'none';
uistack(a, 'bottom');

hold on;

y_points2 = y_points1;
a1 = fill(x_points2, y_points2, 'r');
a1.FaceAlpha = 0.1;
a1.EdgeColor = 'none';
uistack(a1, 'bottom');

h1 = plot(time_SMC, etha_ref_SMC - sf_etha_SMC, 'b', 'LineWidth', linewidth); hold on; 
h2 = plot(time_STSM, etha_ref_STSM - sf_etha_STSM, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', linewidth); hold on; 
grid on;

xlabel('Time [s]');
title('Evolution of Tracking Error in $\eta$-loop [deg]', 'Interpreter', 'latex');
%set(gca, 'FontSize', digitsFont);

% Axis limits
axis([x1 x2 y1 y2]); % 4m/s

exportgraphics(gca,'Tracking_Error_4ms_eta_STSM_SMC.pdf','ContentType','vector')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


fig = figure('Name', 'Tracking Error beta', 'NumberTitle', 'off');

y1 = -8; y2 = 5;
y_points1 = [y1, y2, y2, y1];
a = fill(x_points1, y_points1, 'b');
a.FaceAlpha = 0.1;
a.EdgeColor = 'none';
uistack(a, 'bottom');

hold on;

y_points2 = y_points1;
a1 = fill(x_points2, y_points2, 'r');
a1.FaceAlpha = 0.1;
a1.EdgeColor = 'none';
uistack(a1, 'bottom');

h1 = plot(time_SMC, beta_ref_SMC - sf_beta_SMC, 'b', 'LineWidth', linewidth); hold on; 
h2 = plot(time_STSM, beta_ref_STSM - sf_beta_STSM, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', linewidth); hold on; 
grid on;

xlabel('Time [s]');
title('Evolution of Tracking Error in $\beta$-loop [deg]', 'Interpreter', 'latex');
%set(gca, 'FontSize', digitsFont);

% Axis limits
axis([x1 x2 y1 y2]); % 4m/s

legend([h1, h2],"Baseline Control: SMC", ...
    "Baseline Control: STSMC",'Interpreter', 'latex', 'Location', 'northwest');


exportgraphics(gca,'Tracking_Error_4ms_beta_STSM_SMC.pdf','ContentType','vector')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %% Quadcopter Thrust PLot
% figure('Name','Thrust','NumberTitle','off') ;
% 
% y1 = 14.5; y2 = 19;
% y_points1 = [y1, y2, y2, y1];
% a = fill(x_points1, y_points1, 'b');
% a.FaceAlpha = 0.1;
% a.EdgeColor = 'none';
% uistack(a, 'bottom');
% 
% hold on;
% 
% y_points2 = y_points1;
% a1 = fill(x_points2, y_points2, 'r');
% a1.FaceAlpha = 0.1;
% a1.EdgeColor = 'none';
% uistack(a1, 'bottom');
% 
% h1 = plot(time_SMC, Thrust_ref_N_SMC, 'r', 'LineWidth', linewidth); hold on; 
% h2 = plot(time_STSM, Thrust_ref_N_STSM, 'Color', 'b', 'LineWidth', linewidth); hold on; 
% grid on;
% 
% xlabel('Time [s]');
% title('Evolution of Quadcopter Thrust Force [N]', 'Interpreter', 'latex');
% 
% % Axis limits
% axis([x1 x2 y1 y2]); % 4m/s
% 
% exportgraphics(gca,'Thrust_4ms_STSM_SMC.pdf','ContentType','vector')

%% Pitch Angle PLot
figure('Name','Theta','NumberTitle','off') ;

y1 = -12.5; y2=1; 
y_points1 = [y1, y2, y2, y1];
a = fill(x_points1, y_points1, 'b');
a.FaceAlpha = 0.1;
a.EdgeColor = 'none';
uistack(a, 'bottom');

hold on;


y_points2 = y_points1;
a1 = fill(x_points2, y_points2, 'r');
a1.FaceAlpha = 0.1;
a1.EdgeColor = 'none';
uistack(a1, 'bottom');

h1 = plot(time_SMC, sf_theta_SMC, 'b', 'LineWidth', linewidth); hold on; 
h2 = plot(time_STSM, sf_theta_STSM, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', linewidth); hold on; 
grid on;

xlabel("Time [s]")


title("Evolution of Pitch Angle [deg]", 'Interpreter', 'latex') ;
axis([x1 x2 y1 y2]); % 4m/s

%set(gca,'Fontsize',digitsFont)
exportgraphics(gca, 'Euler_4ms_STSM_SMC.pdf','ContentType','vector')

%% Quadcopter Thrust PLot
figure('Name','Thrust','NumberTitle','off') ;

y1 = 14.5; y2 = 19;
y_points1 = [y1, y2, y2, y1];
a = fill(x_points1, y_points1, 'b');
a.FaceAlpha = 0.1;
a.EdgeColor = 'none';
uistack(a, 'bottom');

hold on;

y_points2 = y_points1;
a1 = fill(x_points2, y_points2, 'r');
a1.FaceAlpha = 0.1;
a1.EdgeColor = 'none';
uistack(a1, 'bottom');

h1 = plot(time_SMC, Thrust_ref_N_SMC, 'b', 'LineWidth', linewidth); hold on; 
h2 = plot(time_STSM, Thrust_ref_N_STSM, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', linewidth); hold on; 
grid on;

xlabel('Time [s]');
title('Evolution of Quadcopter Thrust Force [N]', 'Interpreter', 'latex');

% Axis limits
axis([x1 x2 y1 y2]); % 4m/s

exportgraphics(gca,'Thrust_4ms_STSM_SMC.pdf','ContentType','vector')


%% 3D Plot STSMC
figure;
h1= plot3(sf_rl_STSM(1:15/Ts_STSM).*cos(sf_etha_STSM(1:15/Ts_STSM)*pi/180).*cos(sf_beta_STSM(1:15/Ts_STSM)*pi/180),...
    sf_rl_STSM(1:15/Ts_STSM).*sin(sf_etha_STSM(1:15/Ts_STSM)*pi/180).*cos(sf_beta_STSM(1:15/Ts_STSM)*pi/180),...
    sf_rl_STSM(1:15/Ts_STSM).*sin(sf_beta_STSM(1:15/Ts_STSM)*pi/180), 'Color', 'b', 'LineWidth', linewidth) ; hold on ; 

h2 = plot3(sf_rl_STSM(15/Ts_STSM:25/Ts_STSM).*cos(sf_etha_STSM(15/Ts_STSM:25/Ts_STSM)*pi/180).*cos(sf_beta_STSM(15/Ts_STSM:25/Ts_STSM)*pi/180), ...
    sf_rl_STSM(15/Ts_STSM:25/Ts_STSM).*sin(sf_etha_STSM(15/Ts_STSM:25/Ts_STSM)*pi/180).*cos(sf_beta_STSM(15/Ts_STSM:25/Ts_STSM)*pi/180), ...
    sf_rl_STSM(15/Ts_STSM:25/Ts_STSM).*sin(sf_beta_STSM(15/Ts_STSM:25/Ts_STSM)*pi/180), 'Color', 'g', 'LineWidth', linewidth) ; hold on ; 

h3 = plot3(sf_rl_STSM(25/Ts_STSM:end).*cos(sf_etha_STSM(25/Ts_STSM:end)*pi/180).*cos(sf_beta_STSM(25/Ts_STSM:end)*pi/180),...
    sf_rl_STSM(25/Ts_STSM:end).*sin(sf_etha_STSM(25/Ts_STSM:end)*pi/180).*cos(sf_beta_STSM(25/Ts_STSM:end)*pi/180), ...
    sf_rl_STSM(25/Ts_STSM:end).*sin(sf_beta_STSM(25/Ts_STSM:end)*pi/180), 'Color', 'r', 'LineWidth', linewidth) ; hold on ; 

h4 = plot3(r_ref_STSM(1:15/Ts_STSM).*cos(etha_ref_STSM(1:15/Ts_STSM)*pi/180).*cos(beta_ref_STSM(1:15/Ts_STSM)*pi/180), ...
    r_ref_STSM(1:15/Ts_STSM).*sin(etha_ref_STSM(1:15/Ts_STSM)*pi/180).*cos(beta_ref_STSM(1:15/Ts_STSM)*pi/180),...
    r_ref_STSM(1:15/Ts_STSM).*sin(beta_ref_STSM(1:15/Ts_STSM)*pi/180),...
    '--k', 'LineWidth', linewidth);
grid on ;

legend([h1, h2, h3, h4],"Take-off Phase", ...
    "Hovering Phase", "Landing Phase", "Desired Trajectory", 'Interpreter', 'latex', 'Location', 'northwest');
xlabel("$p_x$ [m]", 'Interpreter', 'latex')
ylabel("$p_y$ [m]", 'Interpreter', 'latex')
zlabel("$p_z$ [m]", 'Interpreter', 'latex')
% view(-38.3710,  8.7786)
% axis([0.4 1.2 -0.5 0.7 0 1.2])
view(38.8064,  6.3359)
axis([0.4 1.2 -0.5 0.7 0.1 1.2])
title("Evolution of 3D Trajectory Tracking (STSMC)", 'Interpreter', 'latex') ;
exportgraphics(gca, 'threeD_4ms_SMC_STSM.pdf','ContentType','vector')
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%ù

%% 3D Plot SMC
figure;
h1= plot3(sf_rl_SMC(1:15/Ts_SMC).*cos(sf_etha_SMC(1:15/Ts_SMC)*pi/180).*cos(sf_beta_SMC(1:15/Ts_SMC)*pi/180),...
    sf_rl_SMC(1:15/Ts_SMC).*sin(sf_etha_SMC(1:15/Ts_SMC)*pi/180).*cos(sf_beta_SMC(1:15/Ts_SMC)*pi/180),...
    sf_rl_SMC(1:15/Ts_SMC).*sin(sf_beta_SMC(1:15/Ts_SMC)*pi/180), 'Color', 'b', 'LineWidth', linewidth) ; hold on ; 

h2 = plot3(sf_rl_SMC(15/Ts_SMC:25/Ts_SMC).*cos(sf_etha_SMC(15/Ts_SMC:25/Ts_SMC)*pi/180).*cos(sf_beta_SMC(15/Ts_SMC:25/Ts_SMC)*pi/180), ...
    sf_rl_SMC(15/Ts_SMC:25/Ts_SMC).*sin(sf_etha_SMC(15/Ts_SMC:25/Ts_SMC)*pi/180).*cos(sf_beta_SMC(15/Ts_SMC:25/Ts_SMC)*pi/180), ...
    sf_rl_SMC(15/Ts_SMC:25/Ts_SMC).*sin(sf_beta_SMC(15/Ts_SMC:25/Ts_SMC)*pi/180), 'Color', 'g', 'LineWidth', linewidth) ; hold on ; 

h3 = plot3(sf_rl_SMC(25/Ts_SMC:end).*cos(sf_etha_SMC(25/Ts_SMC:end)*pi/180).*cos(sf_beta_SMC(25/Ts_SMC:end)*pi/180),...
    sf_rl_SMC(25/Ts_SMC:end).*sin(sf_etha_SMC(25/Ts_SMC:end)*pi/180).*cos(sf_beta_SMC(25/Ts_SMC:end)*pi/180), ...
    sf_rl_SMC(25/Ts_SMC:end).*sin(sf_beta_SMC(25/Ts_SMC:end)*pi/180), 'Color', 'r', 'LineWidth', linewidth) ; hold on ; 

h4 = plot3(r_ref_SMC(1:15/Ts_SMC).*cos(etha_ref_SMC(1:15/Ts_SMC)*pi/180).*cos(beta_ref_SMC(1:15/Ts_SMC)*pi/180), ...
    r_ref_SMC(1:15/Ts_SMC).*sin(etha_ref_SMC(1:15/Ts_SMC)*pi/180).*cos(beta_ref_SMC(1:15/Ts_SMC)*pi/180),...
    r_ref_SMC(1:15/Ts_SMC).*sin(beta_ref_SMC(1:15/Ts_SMC)*pi/180),...
    '--k', 'LineWidth', linewidth);
grid on ;

legend([h1, h2, h3, h4],"Take-off Phase", ...
    "Hovering Phase", "Landing Phase", "Desired Trajectory", 'Interpreter', 'latex', 'Location', 'northwest');
xlabel("$p_x$ [m]", 'Interpreter', 'latex')
ylabel("$p_y$ [m]", 'Interpreter', 'latex')
zlabel("$p_z$ [m]", 'Interpreter', 'latex')
% view(-38.3710,  8.7786)
% axis([0.4 1.2 -0.5 0.7 0 1.2])
view(38.8064,  6.3359)
axis([0.4 1.2 -0.5 0.7 0.1 1.2])
title("Evolution of 3D Trajectory Tracking (SMC)", 'Interpreter', 'latex') ;
exportgraphics(gca, 'threeD_4ms_SMC_FL.pdf','ContentType','vector')
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Tension Plot
figure('Name','Tension','NumberTitle','off') ;

y1 = -1; y2 = 7;
y_points1 = [y1, y2, y2, y1];
a = fill(x_points1, y_points1, 'b');
a.FaceAlpha = 0.1;
a.EdgeColor = 'none';
uistack(a, 'bottom');

hold on;

y_points2 = y_points1;
a1 = fill(x_points2, y_points2, 'r');
a1.FaceAlpha = 0.1;
a1.EdgeColor = 'none';
uistack(a1, 'bottom');

h1 = plot(time_SMC, Tension_Magnitude_SMC, 'b', 'LineWidth', linewidth); hold on; 
h2 = plot(time_STSM, Tension_Magnitude_STSM, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', linewidth); hold on; 
grid on;

xlabel('Time [s]');
title('Evolution of Tether Traction Force [N]', 'Interpreter', 'latex');
legend([h1, h2],"Baseline Control: SMC", ...
    "Baseline Control: STSMC",'Interpreter', 'latex', 'Location', 'northwest');

% Axis limits
axis([x1 x2 y1 y2]); % 4m/s

exportgraphics(gca,'Tension_4ms_STSM_SMC.pdf','ContentType','vector')

%% Wind Plot
figure('Name','Wind','NumberTitle','off') ;

y1 = 0; y2 = 4.5;
y_points1 = [y1, y2, y2, y1];
a = fill(x_points1, y_points1, 'b');
a.FaceAlpha = 0.1;
a.EdgeColor = 'none';
uistack(a, 'bottom');

hold on;

y_points2 = y_points1;
a1 = fill(x_points2, y_points2, 'r');
a1.FaceAlpha = 0.1;
a1.EdgeColor = 'none';
uistack(a1, 'bottom');

h1 = plot(time_STSM, wind_STSM(:,1), 'Color', 'b', 'LineWidth', linewidth); hold on;
h2 = plot(time_STSM, wind_STSM(:,2), 'Color', 'r', 'LineWidth', linewidth); hold on; 
h3 = plot(time_STSM, wind_STSM(:,3), 'Color', 'g', 'LineWidth', linewidth); hold on; 
grid on;

xlabel('Time [s]');
title('Evolutaion of Wind Velocity [m.s$^{-1}$]', 'Interpreter', 'latex');
legend([h1, h2, h3],"Wind speed in forward direction", ...
    "Wind speed in sideways direction", "Wind speed in vertical direction",'Interpreter', 'latex', 'Location', 'northeast');

% Axis limits
axis([x1 x2 y1 y2]); % 4m/s

exportgraphics(gca,'Wind_4ms_STSM_SMC.pdf','ContentType','vector')






%% Peroformance of Sliding Surface Plot
figure('Name','Sliding Surface r-loop','NumberTitle','off') ;

y1 = -0.2; y2 = 0.4;
y_points1 = [y1, y2, y2, y1];
a = fill(x_points1, y_points1, 'b');
a.FaceAlpha = 0.1;
a.EdgeColor = 'none';
uistack(a, 'bottom');

hold on;

y_points2 = y_points1;
a1 = fill(x_points2, y_points2, 'r');
a1.FaceAlpha = 0.1;
a1.EdgeColor = 'none';
uistack(a1, 'bottom');

h1 = plot(time_SMC, S_SMC_SMC(:,1), 'b', 'LineWidth', linewidth); hold on; 
h2 = plot(time_STSM, S_SMC_STSM (:,1), 'Color', [0.4660 0.6740 0.1880], 'LineWidth', linewidth); hold on;
grid on;


xlabel('Time [s]');
title('Evolution of Radial Sliding Surface $s_r$', 'Interpreter', 'latex');

% Axis limits
axis([x1 x2 y1 y2]); % 4m/s

exportgraphics(gca,'ss_r_4ms_STSM_SMC.pdf','ContentType','vector')

%%%%%%%
figure('Name','Sliding Surface eta-loop','NumberTitle','off') ;

y1 = -0.2; y2 = 0.7;
y_points1 = [y1, y2, y2, y1];
a = fill(x_points1, y_points1, 'b');
a.FaceAlpha = 0.1;
a.EdgeColor = 'none';
uistack(a, 'bottom');

hold on;

y_points2 = y_points1;
a1 = fill(x_points2, y_points2, 'r');
a1.FaceAlpha = 0.1;
a1.EdgeColor = 'none';
uistack(a1, 'bottom');

h1 = plot(time_SMC, S_SMC_SMC(:,2), 'b', 'LineWidth', linewidth); hold on; 
h2 = plot(time_STSM, S_SMC_STSM (:,2), 'Color', [0.4660 0.6740 0.1880], 'LineWidth', linewidth); hold on;
grid on;


xlabel('Time [s]');
title('Evolution of Azimuthal Sliding Surface $s_{\eta}$', 'Interpreter', 'latex');


% Axis limits
axis([x1 x2 y1 y2]); % 4m/s

exportgraphics(gca,'ss_eta_4ms_STSM_SMC.pdf','ContentType','vector')

%%%%%%%%
figure('Name','Sliding Surface beta-loop','NumberTitle','off') ;

y1 = -0.2; y2 = 0.4;
y_points1 = [y1, y2, y2, y1];
a = fill(x_points1, y_points1, 'b');
a.FaceAlpha = 0.1;
a.EdgeColor = 'none';
uistack(a, 'bottom');

hold on;

y_points2 = y_points1;
a1 = fill(x_points2, y_points2, 'r');
a1.FaceAlpha = 0.1;
a1.EdgeColor = 'none';
uistack(a1, 'bottom');

h1 = plot(time_SMC, S_SMC_SMC(:,3), 'b', 'LineWidth', linewidth); hold on; 
h2 = plot(time_STSM, S_SMC_STSM(:,3), 'Color', [0.4660 0.6740 0.1880], 'LineWidth', linewidth); hold on;
grid on;

xlabel('Time [s]');
title('Evolution of Polar Sliding Surface $s_{\beta}$', 'Interpreter', 'latex');
legend([h1, h2],"Baseline Control: SMC", ...
    "Baseline Control: STSMC", 'Interpreter', 'latex', 'Location', 'northwest');

% Axis limits
axis([x1 x2 y1 y2]); % 4m/s

exportgraphics(gca,'ss_beta_4ms_STSM_SMC.pdf','ContentType','vector')