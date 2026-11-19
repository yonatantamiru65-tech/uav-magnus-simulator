function quaternion_error = quaternion_error_from_q_qref(input)


q0 = input(1);
q1 = input(2);
q2 = input(3);
q3 = input(4);

q0_ref = input(5);
q1_ref = input(6);
q2_ref = input(7);
q3_ref = input(8);

q_inv = [q0 -q1 -q2 -q3]'/norm([q0 q1 q2 q3],2) ;

quaternion_error = quaternion_product(q_inv, [q0_ref q1_ref q2_ref q3_ref]') ;

end