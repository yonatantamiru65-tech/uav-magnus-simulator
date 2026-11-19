function product = quaternion_product(q, p)

% scalaire = (q(1)*r(1) - [q(2) q(3) q(4)]*[r(2) r(3) r(4)]') ;
% vecteur = (r(1)*[q(2) q(3) q(4)]' + q(1)*[r(2) r(3) r(4)]' + cross([q(2) q(3) q(4)]', [r(2) r(3) r(4)]')) ;              
% 
% product = [scalaire ; vecteur] ;


q0 = q(1) ;
q1 = q(2) ;
q2 = q(3) ;
q3 = q(4) ;

Q_of_q = [q0 -q1 -q2 -q3 ;
          q1 q0 -q3 q2 ;
          q2 q3 q0 -q1 ;
          q3 -q2 q1 q0] ; 
product = Q_of_q*p ;
end

