function [q_tau, coeff] = quintic_poly_double_norm_compute_coeff(qin, qfin, vin, vfin, ain, afin, T, print_info)
    % Compute coefficients of quintic polynomial for joint trajectory
    %
    % Inputs:
    %   qin         : Initial joint angle
    %   qfin        : Final joint angle
    %   vin         : Initial velocity
    %   vfin        : Final velocity
    %   ain         : Initial acceleration
    %   afin        : Final acceleration
    %   T           : Total time
    %   print_info  : Boolean to print detailed information
    %
    % Outputs:
    %   q_tau: Symbolic expression for joint trajectory
    %   coeff: Vector of trajectory coefficients [a, b, c, d, e, f]

    syms tau real
    syms a b c d e f real

    % Define quintic polynomial
    q_tau = a*tau^5 + b*tau^4 + c*tau^3 + d*tau^2 + e*tau + f;
    
    % Define boundary conditions
    eq_1 = subs(q_tau, tau, 0) == qin;
    eq_2 = subs(q_tau, tau, 1) == qfin;
    
    q_tau_diff = diff(q_tau, tau);
    eq_3 = subs(q_tau_diff, tau, 0) == vin*T;
    eq_4 = subs(q_tau_diff, tau, 1) == vfin*T;
    
    q_tau_dd = diff(q_tau_diff, tau);
    eq_5 = subs(q_tau_dd, tau, 0) == ain*T^2;
    eq_6 = subs(q_tau_dd, tau, 1) == afin*T^2;
    
    % Solve for coefficients
    sol = solve([eq_1, eq_2, eq_3, eq_4, eq_5, eq_6], [a, b, c, d, e, f]);
    
    % Extract and simplify coefficients
    coeff = double([sol.a, sol.b, sol.c, sol.d, sol.e, sol.f]);
    
    % Substitute coefficients into q_tau
    q_tau = vpa(subs(q_tau, [a, b, c, d, e, f], coeff), 5);

    if print_info
        fprintf("Quintic Polynomial Double Normalized (tau belongs to [0, 1])\n");
        fprintf("---------------------------------------------------------\n");
        fprintf("Equations:\n");
        disp(eq_1); disp(eq_2); disp(eq_3); disp(eq_4); disp(eq_5); disp(eq_6)
    
        fprintf("---------------------------------------------------------\n");
        fprintf("Symbolic solutions [a, b, c, d, e, f]:\n");
        disp(coeff);
    
        fprintf("---------------------------------------------------------\n");
        fprintf("Normalized trajectory:\n");
        disp(q_tau);
        fprintf("##############################################################\n");
    end
end