function output = rot_y(pitch_angle)
%     """ Perform rotation around x axis by pitch_angle"""
    c_a = cos(pitch_angle) ;
    s_a = sin(pitch_angle) ;
    rot_matrix = [c_a 0 s_a ;
                  0 1 0 ;
                  -s_a 0 c_a] ;
   
              
 output =  rot_matrix ;
end
