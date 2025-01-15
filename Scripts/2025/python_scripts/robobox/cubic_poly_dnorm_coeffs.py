import sympy as sp

def cubic_poly_dnorm_coeffs(qin: float, qfin: float, vin: float,
                            vfin, T: float, verbose: bool=0) -> tuple:
    """
    Computes the coefficients of a cubic polynomial for joint trajectory (doubly normalized)

    @param qin: The initial joint angle
    @param qfin: The final joint angle
    @param vin: The initial velocity
    @param vfin: The final velocity
    @param T: The total time
    @param verbose: The control flag
    """
    tau = sp.symbols('tau', real=True)
    a, b, c, d = sp.symbols('a b c d', real=True)

    # Cubic polynomial
    qn = a*tau**3 + b*tau**2 + c*tau + d
    deltaQ = qfin - qin
    q_tau = qin + deltaQ*qn

    # Boundary conditions
    eq_1 = sp.Eq(d, 0)
    eq_2 = sp.Eq(a + b + c, 1)
    eq_3 = sp.Eq(c, vin*T/deltaQ)
    eq_4 = sp.Eq(3*a + 2*b + c, vfin * T/deltaQ)

    # Solving
    sol = sp.solve([eq_1, eq_2, eq_3, eq_4], [a, b, c, d])
    coeffs = [sol[a], sol[b], sol[c], sol[d]]
    q_tau = q_tau.subs(sol)

    if verbose:
        print("==== Equations ====")
        print(eq_1, eq_2, eq_3, eq_4)
        print("==== Symbolic solutions [a, b, c, d] ====")
        print(coeffs)
        print("==== Normalized trajectory ====")
        print(q_tau)
        
    return q_tau, coeffs
