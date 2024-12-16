function q_tau = cubic_poly_double_norm(a, b, c, d, tau, qin, qfin)
    Dq = qfin-qin;
    qn = a*tau^3 + b*tau^2 +c*tau + d;
    q_tau = qin + Dq*qn;
end

