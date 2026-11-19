function [Fa_LB,Fa_G] = Aero_initial(Va,data, wmag, quad)
CDy=data(1);
Slat=data(2);
S=data(3);
R=data(4);
F_a_y_local = (1/2)*CDy*Slat*abs(Va(2))*Va(2);
Vaxz=norm([Va(1) Va(3)]);
if Vaxz==0
     eFD=[1 0 0];
     X1=0;
else
     eFD=[Va(1)/Vaxz 0 Va(3)/Vaxz];
     X1= min(wmag*R/Vaxz,7);%saturation car validité du modele à X=<7
end

eFL=cross([0 1 0],eFD);
X1  = min(X1,8);
CD  = (-0.0211*X1.^3+0.1873*X1.^2+0.1183*X1+0.5);
CL  = 0.0126*X1.^4-0.2004*X1.^3+0.7482*X1.^2+1.3447*X1;

F_L_xz_local = (1/2)*CL*S*Vaxz^2*eFL;
F_D_xz_local = (1/2)*CD*S*Vaxz^2*eFD;
Fa_LB = [0 F_a_y_local 0]+F_D_xz_local+F_L_xz_local;
q0=quad(1);q1=quad(2);q2=quad(3);q3=quad(4);
R_matrix = [(q0^2 + q1^2 - q2^2 - q3^2)  2*(q1*q2 - q0*q3) 2*(q1*q3 + q0*q2) ;
                 2*(q1*q2 + q0*q3)  (q0^2 - q1^2 + q2^2 - q3^2) 2*(q2*q3 - q0*q1) ;
                 2*(q1*q3 - q0*q2) 2*(q2*q3 + q0*q1) (q0^2 - q1^2 - q2^2 + q3^2) ] ;
Fa_G1=R_matrix*Fa_LB';
Fa_G=Fa_G1;
end 