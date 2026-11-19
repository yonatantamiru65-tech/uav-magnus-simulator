function [c, ceq] = myNonlinearConstraints(x, r_ddot, r_dot, m_w, u_total_const, T0)
    c = 1.5 - (m_w*r_ddot + x(1) + 0.4*tanh(60*r_dot));
    ceq = x(2) - x(1) - u_total_const;

end