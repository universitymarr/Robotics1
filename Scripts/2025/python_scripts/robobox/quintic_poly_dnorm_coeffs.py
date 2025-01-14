import sympy as sp

def quintic_poly_dnorm_coeffs(qin: float, qfin: float, vin: float, vfin: float, 
                                           ain: float, afin: float, T: float, verbose: bool = 0) -> tuple:
    """
    Computes the coefficients of a quintic polynomial for joint trajectory (doubly normalized)

    @param qin: The initial joint angle
    @param qfin: The Final joint angle
    @param vin: The initial velocity
    @param vfin: The final velocity
    @param ain: The initial acceleration
    @param afin: The final acceleration
    @param T: The total time
    @param verbose: The control flag
    """
    tau = sp.symbols('tau', real=True)
    a, b, c, d, e, f = sp.symbols('a b c d e f', real=True)

    # Quintic polynomial
    q_tau = a*tau**5 + b*tau**4 + c*tau**3 + d*tau**2 + e*tau + f
    
    # Boundary conditions
    eq_1 = sp.Eq(q_tau.subs(tau, 0), qin)
    eq_2 = sp.Eq(q_tau.subs(tau, 1), qfin)
    
    q_tau_diff = sp.diff(q_tau, tau)
    eq_3 = sp.Eq(q_tau_diff.subs(tau, 0), vin*T)
    eq_4 = sp.Eq(q_tau_diff.subs(tau, 1), vfin*T)
    
    q_tau_dd = sp.diff(q_tau_diff, tau)
    eq_5 = sp.Eq(q_tau_dd.subs(tau, 0), ain*T**2)
    eq_6 = sp.Eq(q_tau_dd.subs(tau, 1), afin*T**2)
    
    # Solving 
    sol = sp.solve([eq_1, eq_2, eq_3, eq_4, eq_5, eq_6], [a, b, c, d, e, f])
    coeffs = [sol[a], sol[b], sol[c], sol[d], sol[e], sol[f]]
    q_tau = q_tau.subs(sol)

    if verbose:
        print("==== Equations ====")
        print(eq_1, eq_2, eq_3, eq_4, eq_5, eq_6)
        print("==== Symbolic solutions [a, b, c, d, e, f] ====")
        print(coeffs)
        print("==== Normalized trajectory ====")
        print(q_tau)
        
    return q_tau, coeffs
