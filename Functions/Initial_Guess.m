function X0  = Initial_Guess(data)
Va=data(1:3);
comp=data(4:9);
CDy=comp(1); 
Slat=comp(2);
S=comp(3);
max_angle=comp(4);
m=comp(5);
g=comp(6);


Va_xz=[Va(1), 0, Va(3)];
Va=norm(Va_xz);
if Va ~=0
    if Va>=7.176 && Va<=7.564
        X0=(3.9301-5.14)/(7.564-7.1761)*Va+5.14-(3.9301-5.14)/(7.564-7.1761)*7.1761;
        %X0=3.9326;
    else 
        X=0:0.01:8;
        CL  = (0.0126*X.^4-0.2004*X.^3+0.7482*X.^2+1.3447*X);
        F_L= (1/2)*CL*S*Va^2;
        [FL_max,v]=max(F_L);
        X0=double(X(v));
        X0=X0(X0>0);
        CD  = (-0.0211*X0.^3+0.1873*X0.^2+0.1183*X0+0.5);
        FD_max= (1/2)*CD*S*Va^2;
        if FD_max/(m*g-FL_max)>= tan(max_angle) || FD_max/(m*g-FL_max)<=tan(-max_angle)
            syms X
            disp('yes')
            CD  = (-0.0211*X.^3+0.1873*X.^2+0.1183*X+0.5);
            CL  = (0.0126*X.^4-0.2004*X.^3+0.7482*X.^2+1.3447*X);
            F_D= (1/2)*CD*S*Va^2;
            F_L= (1/2)*CL*S*Va^2;
            X0=double(vpa(solve(F_D/(m*g-F_L)-tan(max_angle),X,'Real',true)));
        end
        X0=X0(X0>0);
    end 
else 
    X0=0;
end
end 