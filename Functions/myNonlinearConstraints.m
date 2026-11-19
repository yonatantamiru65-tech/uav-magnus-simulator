function [c, ceq] = myNonlinearConstraints(x, r_ddot_const, m_w_const, u_total_const, T0)
    c = [T0 - (m_w_const*r_ddot_const + x(1) + 0.6)];
    ceq = [x(2) - x(1) - u_total_const];

end