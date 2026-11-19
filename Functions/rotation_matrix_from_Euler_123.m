function R = rotation_matrix_from_Euler_123(input) 

    phi = input(1) ;
    theta = input(2) ;
    psi = input(3) ;


    R = [ cos(theta)*cos(psi)    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)          cos(phi)*sin(theta)*cos(psi)+sin(psi)*sin(phi) ;
         cos(theta)*sin(psi)        sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)        cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) ;
           - sin(theta)                     sin(phi)*cos(theta)                                 cos(phi)*cos(theta) ] ;


end 
