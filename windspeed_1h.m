clc
y_min = 5;
y_max = 25;
% 
% lines = current_state ; rows = next_state
P = [0.81383 0.179521 0.006649 0 0 0 0 0;
0.200573 0.530086 0.219198 0.035817 0.014327 0 0 0;
0.200573 0.530086 0.219198 0.035817 0.014327 0 0 0;
0 0.211043 0.541104 0.141104 0.045399 0.051534 0.008589 0.001227;
0 0 0.068273 0.269076 0.398929 0.195448 0.068273 0;
0 0 0.009576 0.135431 0.228454 0.385773 0.184679 0.056088; 
0 0 0 0 0.037783 0.248111 0.578086 0.13602;
0 0 0 0 0 0.015421 0.161329 0.82325];

% lines = current_state ; rows = next_state
T = [0.9 0.1;
    0.9 0.1];

param = [0.1619 0.2878 7.5988 0.4337;
         0.0855 0.1700 8.0542 0.4310;
         0.0314 0.2573 10.0245 0.6459;
         0.1049 0.3137 10.2984 0.6373;
         0.0459 0.5118 10.6661 1.2831;
         0.0196 0.2901 11.2078 0.8246;
         0.0683 0.4051 12.4011 0.9461;
         0.0957 0.3008 12.9833 0.5577];
     

% Chain parameters


chain_length_T = 6; %6    %number of times I want to change state
chain_T = zeros(1,chain_length_T);         %comenzamos en el estado 1
starting_value = 1;
chain_T(1) = starting_value;


% Time parameters
dt = 0.002;          % interval between samplings
time_in_T = 600; % seconds in T before change state (10 min)
steps_in_T = length(0:dt:time_in_T)-1;
tmax =chain_length_T * time_in_T; %total time to simualte
t  = 0:dt:tmax;
N  = length(t);

dW = sqrt(dt) .* randn(N,1); %From de original model

%y(1) = 10*rand(1,1);
y(1)=9;

hold on
 current_state_T = randi(size(T,1));
 for j = 1:chain_length_T

        if current_state_T == 1
            for k = 1:steps_in_T
               
                current_index = k + (j-1)*steps_in_T;
                
                y(current_index+1) = y(current_index) -param(current_state_P,1)*(y(current_index))*dt + param(current_state_P,2)*dW(current_index);            
                
                y(current_index+1) = max(y_min, min(y(current_index+1), y_max));
                 
            end
           
        elseif current_state_T == 2
            for k = 1:steps_in_T
                current_index = k + (j-1)*steps_in_T;
                 
                y(current_index+1) =y(current_index)  -(y(current_index)-param(current_state_P,3))*dt + param(current_state_P,4)*dW(current_index); %caso 2
                
                y(current_index+1) = max(y_min, min(y(current_index+1), y_max));
            end
        end
        %xline(current_index, ':r')
        z = rand();
        % primer elemento donde la suma acumulada de T sea mayor que z    
        current_state_T = find(cumsum(T(current_state_T, :)) > z, 1);  

 end
% 
plot (y); xlabel ('Time (s)'); ylabel ('Wind Speed (m/s)'); 
datavals=y(1:steps_in_T*chain_length_T);
datasim=y(1:20001); % number of simulated data, since not all the 1hr is used in the simulation is only used final_time