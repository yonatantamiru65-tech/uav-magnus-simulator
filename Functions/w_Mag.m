function out = w_Mag(Va, data, quad, eta, beta, ubeta, uT,ueta)
CDy=data(1); 
Slat=data(2);
S=data(3);
RM=data(4);
yalmip('clear')
X= sdpvar(1,1); %Spin ratio of the Magnus as disiion variable

F_a_y_local = (1/2)*CDy*Slat*abs(Va(2))*Va(2);
Vaxz=norm([Va(1) Va(3)]);
if Vaxz==0
     eFD=[1 0 0];
else
     eFD=[Va(1)/Vaxz 0 Va(3)/Vaxz];
end

eFL=cross([0 1 0],eFD);
CL = (1 - 1./(1 + exp(-0.904 * X))).*(0.0276*X^4+4.6595*X^3-6.0369*X^2+2.9557*X) +  1./(1 + exp(-0.904 * X)) .*(1.0849*X);
CD = -0.0461 * X^3 + 0.5167 * X^2 -0.6511 * X +0.9081;

F_L_xz_local = (1/2)*CL*S*Vaxz^2*eFL;
F_D_xz_local = (1/2)*CD*S*Vaxz^2*eFD;
Fa_LB = [0 F_a_y_local 0]+F_D_xz_local+F_L_xz_local;
q0=quad(1);q1=quad(2);q2=quad(3);q3=quad(4);
R_matrix = [(q0^2 + q1^2 - q2^2 - q3^2)  2*(q1*q2 - q0*q3) 2*(q1*q3 + q0*q2) ;
                 2*(q1*q2 + q0*q3)  (q0^2 - q1^2 + q2^2 - q3^2) 2*(q2*q3 - q0*q1) ;
                 2*(q1*q3 - q0*q2) 2*(q2*q3 + q0*q1) (q0^2 - q1^2 - q2^2 + q3^2) ] ;
Fa_G1=R_matrix*Fa_LB';
Fa_G=Fa_G1;
R_Matrix_L=[cos(eta)*cos(beta) sin(eta)*cos(beta) sin(beta);
-sin(eta) cos(eta) 0;
-cos(eta)*sin(beta) -sin(eta)*sin(beta) cos(beta)];

Fa_L=R_Matrix_L*Fa_G;
Fa_T=Fa_L(1);
Fa_eta=Fa_L(2);
Fa_beta=Fa_L(3);
cost_function=(ueta-Fa_eta)^2+(ubeta-Fa_beta)^2;
problem=[X>=0, X<=8];%,Fa_T<=uT]; % constraints that have to be satisfied
solution=optimize(problem,cost_function);
T_sqr=double((uT-Fa_T)^2+(ueta-Fa_eta)^2+(ubeta-Fa_beta)^2);
wmag=double(X*Vaxz/RM);
out=[wmag,double(X)];

end 




% function X = w_Mag(Vac, data,quadc, etac, betac, ubetac, uTc,uetac,t)
%  CDy=data(1); Slat=data(2); S=data(3); RM=data(4);
%  persistent Controller
% if t==0
%         %yalmip('clear')
%          sdpvar Vax Vay Vaz 
%         sdpvar eta be ubeta uT ueta 
%         sdpvar q0 q1 q2 q3
% 
%         X= sdpvar(1,1); %Spin ratio of the Magnus as disiion variable
%         constraints=[];%constraints=[X>=0, X<=8]; % constraints that have to be satisfied
%         %options=sdpsettings('verbose',1); %options=sdpsettings('solver','mosek','verbose',1); %options for optimisation
%         Vaxz=Vax^2+Vaz^2;%norm([Vax Vaz]);
%         F_a_y_local = (1/2)*CDy*Slat*abs(Vay)*Vay;
%         eFD=[Vax/Vaxz 0 Vaz/Vaxz];
% 
%         eFL=cross([0 1 0],eFD);
%         CD  = (-0.0211*X^3+0.1873*X^2+0.1183*X+0.5);
%         CL  = (0.0126*X^4-0.2004*X^3+0.7482*X^2+1.3447*X); 
%         F_L_xz_local=((1/2)*CL*S*Vaxz^2)*eFL;
%         F_D_xz_local=((1/2)*CD*S*Vaxz^2)*eFD;
%         Fa_LB = [0 F_a_y_local 0]+F_D_xz_local+F_L_xz_local;
%         R_matrix = [(q0^2 + q1^2 - q2^2 - q3^2)  2*(q1*q2 - q0*q3) 2*(q1*q3 + q0*q2) ;
%                          2*(q1*q2 + q0*q3)  (q0^2 - q1^2 + q2^2 - q3^2) 2*(q2*q3 - q0*q1) ;
%                          2*(q1*q3 - q0*q2) 2*(q2*q3 + q0*q1) (q0^2 - q1^2 - q2^2 + q3^2) ] ;
%         Fa_G1=R_matrix*Fa_LB';
%         Fa_G=Fa_G1;
%         R_Matrix_L=[cos(eta)*cos(be) sin(eta)*cos(be) sin(be);
%         -sin(eta) cos(eta) 0;
%         -cos(eta)*sin(be) -sin(eta)*sin(be) cos(be)];
% 
%         Fa_L=R_Matrix_L*Fa_G;
%         Fa_T=Fa_L(1);
%         Fa_eta=Fa_L(2);
%         Fa_beta=Fa_L(3);
%         objective=(uT-Fa_T)^2+(ueta-Fa_eta)^2+(ubeta-Fa_beta)^2;
%         Controller=optimizer(constraints,objective,[],{Vax,Vay,Vaz,q0,q1,q2,q3,eta,be,ubeta,ueta,uT},X);
%         X = Controller(Vac(1),Vac(2),Vac(3),quadc(1),quadc(2),quadc(3),quadc(4),etac,betac,ubetac,uetac,uTc);
% else 
%         X = Controller(Vac(1),Vac(2),Vac(3),quadc(1),quadc(2),quadc(3),quadc(4),etac,betac,ubetac,uetac,uTc);
% end 
% end 
