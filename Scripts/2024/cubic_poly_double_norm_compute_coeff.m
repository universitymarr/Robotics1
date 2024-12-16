function [q_tau, coeff] = cubic_poly_double_norm_compute_coeff(qin, qfin, vin, vfin, T, print_info)
    % Compute coefficients of cubic polynomial for joint trajectory (double normalized)
    %
    % Inputs:
    %   qin         : Initial joint angle
    %   qfin        : Final joint angle
    %   vin         : Initial velocity
    %   vfin        : Final velocity
    %   T           : Total time
    %   print_info  : Boolean to print detailed information
    %
    % Outputs:
    %   qn: Symbolic expression for normalized joint trajectory
    %   coeff: Vector of trajectory coefficients [a, b, c, d]

    syms tau real
    syms a b c d real

    % Define cubic polynomial
    qn = a*tau^3 + b*tau^2 + c*tau + d;
    deltaQ = qfin - qin;

    q_tau = qin + deltaQ * qn;

    % Define boundary conditions
    eq_1 = d == 0;
    eq_2 = a + b + c == 1;

    eq_3 = c == vin*T/deltaQ;
    eq_4 = 3*a + 2*b + c == vfin*T/deltaQ;

    % Solve for coefficients
    disp('eq_1:'); disp(eq_1);
    disp('eq_2:'); disp(eq_2);
    disp("vin*T/deltaQ:"); disp(vin*T/deltaQ);
    disp('eq_3:'); disp(eq_3);
    disp('eq_4:'); disp(eq_4);
    s = solve([eq_1, eq_2, eq_3, eq_4], [a, b, c, d]);
    
    % Extract and simplify coefficients
    coeff = [s.a, s.b, s.c, s.d];
    
    % Substitute coefficients into qn
    q_tau = subs(q_tau, [a, b, c, d], coeff);

    if print_info
        fprintf("Cubic Polynomial Double Normalized (tau belongs to [0, 1])\n");
        fprintf("---------------------------------------------------------\n");
        fprintf("Where deltaQ = (qfin - qin)\n");
        
        fprintf("---------------------------------------------------------\n");
        fprintf("Equations:\n");
        disp(eq_1); disp(eq_2); disp(eq_3); disp(eq_4);
    
        fprintf("---------------------------------------------------------\n");
        fprintf("Symbolic solutions [a, b, c, d]:\n");
        disp(coeff);
    
        fprintf("---------------------------------------------------------\n");
        fprintf("Normalized trajectory:\n");
        disp(q_tau);
        fprintf("##############################################################\n");
    end
end