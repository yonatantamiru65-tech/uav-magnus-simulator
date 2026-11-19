function A = f_jacobian(x, thrust, euler_angles, m , m_w, g )


phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);

A = zeros(9,9);

A(1,1) = 0;
A(1,2) = 1;
A(1,3) = 0;
A(1,4) = 0; 
A(1,5) = 0;
A(1,6) = 0;
A(1,7) = 0;
A(1,8) = 0;
A(1,9) = 0;

A(2,1) = m/(m+m_w)*(x(4)^2*(cos(x(5)))^2+x(6)^2);
A(2,2) = 0;
A(2,3) = 0;
A(2,4) = m/(m+m_w)*(2*x(1)*x(4)*(cos(x(5)))^2);
A(2,5) = m/(m+m_w)*(-g*cos(x(5))-x(1)*x(4)^2*2*sin(x(5))*cos(x(5)));
A(2,6) = m/(m+m_w)*(2*x(1)*x(6));
A(2,7) = 1/(m+m_w);
A(2,8) = 0;
A(2,9) = 0;
A(2,3) = A(2,3) + 1/(m+m_w)*thrust*(-sin(x(3))*cos(x(5))*cos(psi)*cos(phi)*sin(theta)-cos(x(3))*cos(x(5))*sin(phi)*cos(psi));%%
A(2,5) = A(2,5) + 1/(m+m_w)*thrust*(-cos(x(3))*sin(x(5))*cos(psi)*cos(phi)*sin(theta)+sin(x(3))*sin(x(5))*sin(phi)*cos(psi)+cos(x(5))*cos(phi)*cos(theta));%%

A(3,1) = 0;
A(3,2) = 0;
A(3,3) = 0;
A(3,4) = 1;
A(3,5) = 0;
A(3,6) = 0;
A(3,7) = 0;
A(3,8) = 0;
A(3,9) = 0;

u_e = thrust*(-sin(x(3))*cos(phi)*sin(theta)*cos(psi)-cos(x(3))*sin(phi)*cos(psi));
f44 = -2*m*x(2)*x(4)*cos(x(5))+x(8)+u_e;% should we have u_etha? 
A(4,1) = -1/(m*x(1)^2*cos(x(5)))*f44;
A(4,2) = 1/(x(1)*cos(x(3)))*(-2*x(4)*cos(x(5)));
A(4,3) = 0;
A(4,4) = 1/(x(1)*cos(x(5)))*(-2*x(2)*cos(x(5))+2*x(1)*x(6)*sin(x(5)));%%
A(4,5) = 1/(x(1))*(2*x(1)*x(4)*x(6))/(cos(x(5))^2+1/m*tan(x(5))/cos(x(5))*(x(8)+u_e)); %% %pay attention to the derivative of the tan and should we have the derivative due to u_etha/cos(beta)?
A(4,6) = 1/(x(1)*cos(x(3)))*(2*x(1)*x(4)*sin(x(5))); %%
A(4,7) = 0;
A(4,8) = 1/(m*x(1)*cos(x(5)));
A(4,9) = 0;
A(4,3) = A(4,3)+1/(m*x(1)*cos(x(5)))*thrust*(sin(x(3))*sin(phi)*cos(psi)-cos(x(3))*cos(phi)*sin(theta)*cos(psi));%%

A(5,1) = 0;
A(5,2) = 0;
A(5,3) = 0;
A(5,4) = 0;
A(5,5) = 0;
A(5,6) = 1;
A(5,7) = 0;
A(5,8) = 0;
A(5,9) = 0;

u_b = thrust*(-sin(x(5))*cos(x(3))*cos(phi)*sin(theta)*cos(psi)+sin(x(5))*sin(x(3))*sin(phi)*cos(psi)+cos(x(5))*cos(phi)*cos(theta));
f66 = 2*x(2)*x(6)+g*cos(x(5))-x(9)/m-u_b/m;%% %should it contain also the u_beta?
A(6,1) = 1/(x(1)^2)*f66;
A(6,2) = -1/(x(1))*(2*x(6));
A(6,3) = 0;
A(6,4) = -1/(x(1))*(2*x(1)*x(4)*cos(x(5))*sin(x(5)));
A(6,5) = -1/(x(1))*(-x(1)*x(4)^2*(sin(x(5)))^2+x(1)*x(4)^2*(cos(x(5)))^2-g*sin(x(5)));%for the weight force is it the correct sign?
A(6,6) = -1/(x(1))*(2*x(2));%%
A(6,7) = 0;
A(6,8) = 0;
A(6,9) = 1/(m*x(1));
A(6,3) = A(6,3)+1/(m*x(1))*thrust*(sin(x(5))*sin(x(3))*cos(phi)*sin(theta)*cos(psi)+sin(x(5))*cos(x(3))*sin(phi)*cos(psi));%%
A(6,5) = A(6,5)+1/(m*x(1))*thrust*(cos(x(5))*sin(x(3))*sin(phi)*cos(psi)-cos(x(5))*cos(x(3))*cos(phi)*sin(theta)*cos(psi)-sin(x(5))*cos(phi)*cos(theta));%%

A(7,1) = 0;
A(7,2) = 0;
A(7,3) = 0;
A(7,4) = 0;
A(7,5) = 0;
A(7,6) = 0;
A(7,7) = 0;
A(7,8) = 0;
A(7,9) = 0;

A(8,1) = 0;
A(8,2) = 0;
A(8,3) = 0;
A(8,4) = 0;
A(8,5) = 0;
A(8,6) = 0;
A(8,7) = 0;
A(8,8) = 0;
A(8,9) = 0;

A(9,1) = 0;
A(9,2) = 0;
A(9,3) = 0;
A(9,4) = 0;
A(9,5) = 0;
A(9,6) = 0;
A(9,7) = 0;
A(9,8) = 0;
A(9,9) = 0;

end 