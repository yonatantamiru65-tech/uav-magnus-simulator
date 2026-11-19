%% Extract time
t = UAV_DATA.Time;
D = UAV_DATA.Data;

%% ========== UAV_DATA extraction ==========

% Cartesian speed reference
UAV_speed_ref = D(:,1:3);

% Euler reference (roll, pitch, yaw)
UAV_Euler123_ref = D(:,4:6);

% Yaw reference
UAV_psi_ref = UAV_Euler123_ref(:,3);

% Spherical position reference
UAV_sf_pos = D(:,12:14);

% Spherical speed reference
UAV_sf_speed = D(:,15:17);


%% ========== Simulator_Data extraction ==========

t2 = Simulation_Data.time;
S  = Simulation_Data.signals.values;

% Column mapping of Simulator_Data
% (based on screenshot #2)
UAV_pos_spherical   = S(:,1:3);
UAV_speed_spherical = S(:,4:6);

tether_r_dot        = S(:,15);        % "tether_r_ddot"
tether_r_direction  = S(:,16:18);


%% =======================
%       PLOTTING
% =======================

figure;
subplot(3,1,1);
plot(t, UAV_speed_ref); grid on;
title('UAV Speed Reference (Cartesian)');
legend('v_x','v_y','v_z');

subplot(3,1,2);
plot(t, UAV_Euler123_ref); grid on;
title('Euler123 Reference');
legend('roll','pitch','yaw');

subplot(3,1,3);
plot(t, UAV_psi_ref); grid on;
title('Yaw Reference');

figure;
subplot(2,1,1);
plot(t, UAV_sf_pos); grid on;
title('UAV Spherical Position Reference');
legend('r','\theta','\phi');

subplot(2,1,2);
plot(t, UAV_sf_speed); grid on;
title('UAV Spherical Speed Reference');
legend('r dot','theta dot','phi dot');

figure;
subplot(2,1,1);
plot(t2, UAV_pos_spherical); grid on;
title('UAV pos spherical (Simulator\_Data)');
legend('r','theta','phi');

subplot(2,1,2);
plot(t2, UAV_speed_spherical); grid on;
title('UAV speed spherical (Simulator\_Data)');
legend('r dot','theta dot','phi dot');

figure;
subplot(2,1,1);
plot(t2, tether_r_dot); grid on;
title('Tether r dot');

subplot(2,1,2);
plot(t2, tether_r_direction); grid on;
title('Tether direction vector');
legend('e_x','e_y','e_z');
