function output = rot_x(roll_angle)
%     """ Perform rotation around x axis by roll_angle"""
    c_a = cos(roll_angle) ;
    s_a = sin(roll_angle) ;
    rot_matrix = [1 0 0 ;
                  0 c_a -s_a ;
                  0 s_a c_a] ;
   
              
 output =  rot_matrix ;
end
