
function euler = quaternion2euler(quat) 
% function euler = quaternion2euler(q)

euler = zeros(3,1);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
% euler(3) = atan2(2*q1*q2 + 2*q0*q3 , 2 * q0*q0 + 2*q1*q1 - 1);
% euler(2) = asin(-2*q1*q3 + 2*q0*q2);
% euler(1) = atan2(2*q2*q3 + 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1);

 

%  roll (x-axis rotation)
sinr_cosp = 2*(q0*q1 + q2*q3) ;
cosr_cosp = 1 - 2 * (q1*q1 + q2*q2);
roll = atan2(sinr_cosp, cosr_cosp);

%   pitch (y-axis rotation)
sinp = 2 * (q0*q2 - q3*q1);
if (abs(sinp) >= 1) 
    pitch = sing(pi / 2, sinp); %// use 90 degrees if out of range
else
    pitch = asin(sinp);
end
%  yaw (z-axis rotation)
siny_cosp = 2*(q0*q3 + q1*q2);
cosy_cosp = 1 - 2*(q2*q2 + q3*q3);
yaw = atan2(siny_cosp, cosy_cosp);

euler = [roll pitch yaw] ;


end