function output = rot_z(yaw_angle)
%     """ Perform rotation around x axis by pitch_angle"""
    c_a = cos(yaw_angle) ;
    s_a = sin(yaw_angle) ;
    rot_matrix = [c_a -s_a 0 ;
                  s_a c_a 0 ;
                  0 0 1] ;
 
              
 output =  rot_matrix ;
end
