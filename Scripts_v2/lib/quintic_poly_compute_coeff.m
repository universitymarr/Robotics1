function [a, b, c, d, e, f] = quintic_poly_compute_coeff(qin_, qfin_, vin_, vfin_, ain_, afin_, T_, print_info)
    syms tau real
    syms a b c d e f real
    syms qin qfin real
    syms vin vfin real
    syms ain afin real
    syms T real positive
    
    q_tau = a*tau^5 + b*tau^4 + c*tau^3 + d*tau^2 + e*tau + f;
    
    if print_info
        fprintf("##############################################################\n");
        fprintf("Quintic Polynomial (tau belongs to [0, 1])");
        display(q_tau);
    end
    
    %compute coefficients 
    q_tau_0 = simplify(subs(q_tau, {tau}, {0}));
    eq_1 = q_tau_0 == qin;
    
    q_tau_1 = simplify(subs(q_tau, {tau}, {1}));
    eq_2 = q_tau_1 == qfin;
    
    q_tau_diff = diff(q_tau, {tau});
    q_tau_diff_0 = subs(q_tau_diff, {tau}, {0});
    eq_3 = q_tau_diff_0 == vin*T;
    
    q_tau_diff_1 = subs(q_tau_diff, {tau}, {1});
    eq_4 = q_tau_diff_1 == vfin*T;
    
    q_tau_dd = diff(q_tau_diff, {tau});
    q_tau_dd_0 = subs(q_tau_dd, {tau}, {0});
    eq_5 = q_tau_dd_0 == ain*T^2;
    
    q_tau_dd_1 = subs(q_tau_dd, {tau}, {1});
    eq_6 = q_tau_dd_1 == afin*T^2;
    

    
    sol = solve([eq_1, eq_2, eq_3, eq_4, eq_5, eq_6], [a,b,c,d,e,f]);
    a = simplify(sol.a);
    b = simplify(sol.b);
    c = simplify(sol.c);
    d = simplify(sol.d);
    e = simplify(sol.e);
    f = simplify(sol.f);
    
    if print_info
        fprintf("---------------------------------------------------------\n");
        fprintf("Equations:");
        display(eq_1)
        display(eq_2)
        display(eq_3)
        display(eq_4)
        display(eq_5)
        display(eq_6)
        
        fprintf("---------------------------------------------------------\n");
        fprintf("Symbolic solutions:");
        display(a)
        display(b)
        display(c)
        display(d)
        display(e)
        display(f)
    end
    
    %input
    qin = qin_;
    qfin = qfin_;
    
    vin = vin_;
    vfin = vfin_;
    
    ain = ain_;
    afin = afin_;
    
    T = T_;
    

    
    a = double(subs(a));
    b = double(subs(b));
    c = double(subs(c));
    d = double(subs(d));
    e = double(subs(e));
    f = double(subs(f));

    if print_info
        fprintf("---------------------------------------------------------\n");
        fprintf("Numerical solutions:\n");
        display(a)
        display(b)
        display(c)
        display(d)
        display(e)
        display(f)
        fprintf("##############################################################\n");
    end
end