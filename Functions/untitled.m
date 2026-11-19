global CDy Slat S RM m g rho_air
max_Eulerangle = 35*pi/180;
X1 = 0:0.1:8;
V = 0:0.5:12;
[X,Va] = meshgrid(X1, V);
data = [CDy Slat S RM max_Eulerangle m g rho_air];
J = costFunction2(X, Va, data, [0 0 m*g]);

load('DATA_Simulated_bedmodel.mat')
Data = Data_lookupTable;
Va_sim = Data(1,:);
Thrust = Data(2,:);
Theta = Data(3,:);
wmag = Data(4,:);
Xmag = Data(5,:);
FaG_act = Data(6:8,:);
FaG_control = Data(9:11,:);
FG_desired = Data(12:14,:);

figure()
contour(X, Va, J,40)
hold on; yline(7); hold on; yline(9);
grid on 

figure()
contourf(Va, X, J,40)
hold on; xline(7); hold on; xline(9);
hold on; plot(Va_sim, Xmag,'-o','Color','r')
grid on 