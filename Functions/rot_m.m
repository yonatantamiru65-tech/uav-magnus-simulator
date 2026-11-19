function output = rot_m(angle1, angle2, angle3)
%  Tait-Bryan Angles
    rm1 = rot_z(angle1) ;
    rm2 = rot_y(angle2) ;
    rm3 = rot_x(angle3) ;
    rot_matrix = rm1*rm2*rm3 ;
    
    output =  rot_matrix ;
    
end
    