function quaternion = euler2quaternion(input) 


phi = input(1) ;
theta = input(2) ;
psi = input(3) ;

roll = phi ;
pitch = theta ;
yaw = psi ;

cy = cos(yaw * 0.5);
sy = sin(yaw * 0.5);
cp = cos(pitch * 0.5);
sp = sin(pitch * 0.5);
cr = cos(roll * 0.5);
sr = sin(roll * 0.5);

q0 = cr * cp * cy + sr * sp * sy;
q1 = sr * cp * cy - cr * sp * sy;
q2 = cr * sp * cy + sr * cp * sy;
q3 = cr * cp * sy - sr * sp * cy;


quaternion = [q0 q1 q2 q3]  ;

end