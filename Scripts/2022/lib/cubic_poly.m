function q_tau = cubic_poly(a3, a2, a1, a0, tau)
    %NOTE:
    % cubic_splines_norm return a0, a1, a2, a3 instead of a3, a2, a1, a0
    q_tau = a3*tau^3 + a2*tau^2 + a1*tau + a0;
end

