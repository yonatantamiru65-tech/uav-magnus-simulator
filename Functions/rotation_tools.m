function output = rot_m(angle1, angle2, angle3, sequence)
%  Tait-Bryan Angles
    if sequence == 'ZYX':
        rm1 = rot_z(angle1) ;
        rm2 = rot_y(angle2) ;
        rm3 = rot_x(angle3) ;
        rot_matrix = rm1*rm2*rm3 ;
    else
        rot_matrix = eye(3) ;
        disp('erreur') ;
    end
    
    output =  rot_matrix ;