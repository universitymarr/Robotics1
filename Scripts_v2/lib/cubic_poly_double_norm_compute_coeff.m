function [a, b, c, d, e, f] = cubic_poly_double_norm_compute_coeff(qin_, qfin_, vin_, vfin_, T_, print_info)
    syms tau real
    syms a b c d real
    syms qin qfin real
    syms vin vfin real
    syms Dq real
    syms T real positive
    
    qn = a*tau^3 + b*tau^2 +c*tau + d;
    q_tau = qin + Dq*qn;
    
    if print_info
        fprintf("##############################################################\n");
        fprintf("Cubic Polynomial Double Normalized (tau belongs to [0, 1])");
        display(q_tau);
        fprintf("Where Dq = (qfin - qin)");
    end
    
    %compute coefficients 
    eq_1 = d==0;
    eq_2 = a+b+c==1;
    eq_3 = c==vin*T/Dq;
    eq_4 = 3*a+2*b+c == vfin*T/Dq;

    s = solve([eq_1, eq_2, eq_3, eq_4], [a, b, c, d]);
    a = simplify(s.a);
    b = simplify(s.b);
    c = simplify(s.c);
    d = simplify(s.d);
  
    if print_info
        fprintf("---------------------------------------------------------\n");
        fprintf("Equations:");
        display(eq_1);
        display(eq_2);
        display(eq_3);
        display(eq_4);

        fprintf("---------------------------------------------------------\n");
        fprintf("Symbolic solutions:");
        display(a);
        display(b);
        display(c);
        display(d);

    end
    
    %input
    qin = qin_;
    qfin = qfin_;
    
    vin = vin_;
    vfin = vfin_;

    Dq = (qfin - qin);
    T = T_;
    
    %could be useful to use vpa() function to write fraction as decimal number
    a = subs(a);
    b = subs(b);
    c = subs(c);
    d = subs(d);

    if print_info
        fprintf("---------------------------------------------------------\n");
        fprintf("Numerical solutions:\n");
        display(a)
        display(b)
        display(c)
        display(d)
        fprintf("##############################################################\n");
    end
end