%close all

Data=UAV_DATA;
start_time = 0; % Start time of the simulation
stop_time = tf_echelons; % Stop time of the simulation
num_time_steps = length(UAV_DATA); % Number of time steps in the simulation
dt = (stop_time - start_time) / (num_time_steps - 1); % Time step size
time = linspace(start_time, stop_time, num_time_steps);

Pos_speed_ref = Data(:,1:6) ;
Euler123_ref = Data(:,7:9) ;
pqr_ref = Data(:,10:12) ;
state_feedback = Data(:,13:end) ;


x_ref = Pos_speed_ref(:,1) ;
y_ref = Pos_speed_ref(:,2) ;
z_ref = Pos_speed_ref(:,3) ;
x_dot_ref = Pos_speed_ref(:,4) ;
y_dot_ref = Pos_speed_ref(:,5) ;
z_dot_ref = Pos_speed_ref(:,6) ;

phi_ref = Euler123_ref(:,1) ;
theta_ref = Euler123_ref(:,2) ;
psi_ref = Euler123_ref(:,3) ;

p_ref = pqr_ref(:,1) ;
q_ref = pqr_ref(:,2) ;
r_ref = pqr_ref(:,3) ;

sf_x = state_feedback(:,1) ;
sf_y = state_feedback(:,2) ;
sf_z = state_feedback(:,3) ;
sf_x_dot = state_feedback(:,4) ;
sf_y_dot = state_feedback(:,5) ;
sf_z_dot = state_feedback(:,6) ;
sf_phi = state_feedback(:,7) ;
sf_theta = state_feedback(:,8) ;
sf_psi = state_feedback(:,9) ;
sf_p = state_feedback(:,10) ;
sf_q = state_feedback(:,11) ;
sf_r = state_feedback(:,12) ;
sf_q0 = state_feedback(:,12) ;
sf_q1 = state_feedback(:,13) ;
sf_q2 = state_feedback(:,14) ;
sf_q3 = state_feedback(:,15) ;

figure() ;
subplot(311) ; sgtitle("Position response in E") ;
plot(time, x_ref) ; hold on ; plot(time, sf_x) ; grid on ; legend("x_{ref}", "x") , xlabel("Time [s]") ; ylabel("Position [m]") ;
subplot(312) ;
plot(time, y_ref) ; hold on ; plot(time, sf_y) ; grid on ; legend("y_{ref}", "y") , xlabel("Time [s]") ; ylabel("Position [m]") ;
subplot(313) ; 
plot(time, z_ref) ; hold on ; plot(time, sf_z) ; grid on ; legend("z_{ref}", "z") , xlabel("Time [s]") ; ylabel("Position [m]") ;


figure() ;
subplot(311) ; sgtitle("Angular response in E (Euler X-Y-Z)") ;
plot(time, phi_ref) ; hold on ; plot(time, sf_phi) ; grid on ; legend("\phi_{ref}", "\phi") , xlabel("Time [s]") ; ylabel("Attitude [rad]") ;
subplot(312) ;
plot(time, theta_ref) ; hold on ; plot(time, sf_theta) ; grid on ; legend("\theta_{ref}", "\theta") , xlabel("Time [s]") ; ylabel("Attitude [rad]") ;
subplot(313) ; 
plot(time, psi_ref) ; hold on ; plot(time, sf_psi) ; grid on ; legend("\psi_{ref}", "\psi") , xlabel("Time [s]") ; ylabel("Attitude [rad]") ;


figure() ;
subplot(311) ; sgtitle("Angular velocities response in E (Euler X-Y-Z)") ;
plot(time, p_ref) ; hold on ; plot(time, sf_p) ; grid on ; legend("p_{ref}", "p") , xlabel("Time [s]") ; ylabel("Angular speed [rad.s{-1}") ;
subplot(312) ;
plot(time, q_ref) ; hold on ; plot(time, sf_q) ; grid on ; legend("q_{ref}", "q") , xlabel("Time [s]") ; ylabel("Angular speed [rad.s{-1}") ;
subplot(313) ; 
plot(time, r_ref) ; hold on ; plot(time, sf_r) ; grid on ; legend("r_{ref}", "r") , xlabel("Time [s]") ; ylabel("Angular speed [rad.s{-1}") ;





figure() ;
title("3D Position response in E") ;
plot3(sf_x, sf_y, sf_z) ; hold on ; grid on ; plot3(x_ref, y_ref, z_ref) ; 
legend("Measured trajectory", "Desired trajectory") , xlabel("Position x [m]") ; ylabel("Position y [m]") ; zlabel("Position z [m]") ;


figure(1)