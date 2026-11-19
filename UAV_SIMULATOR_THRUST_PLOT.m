close all 

Data=UAV_SIMULATOR_THRUST;
start_time = 0; % Start time of the simulation
stop_time = tf_echelons; % Stop time of the simulation
num_time_steps = length(UAV_SIMULATOR_THRUST); % Number of time steps in the simulation
dt = (stop_time - start_time) / (num_time_steps - 1); % Time step size
time = linspace(start_time, stop_time, num_time_steps);

Thrust=Data(:,1);
Thrust_ref=Data(:,2);
plot(time, Thrust_ref) ; hold on ; plot(time, Thrust) ; grid on ; legend("Thrust_{ref}", "Thrust") , xlabel("Time [s]") ; ylabel("Thrust [N]") ;
