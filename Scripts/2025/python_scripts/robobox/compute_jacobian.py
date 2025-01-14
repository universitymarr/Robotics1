import sympy as sp

def compute_jacobian(functions, variables: list) -> sp.Matrix:
    """
    Computes the Jacobian matrix of a symbolic function

    @param fonctions : The functions list or the matrix
    @param variables : The variables list
    """
    # Converts the function list to a matrix
    if isinstance(functions, list):
        functions = sp.Matrix(functions)
    
    J = functions.jacobian(variables)
    
    return J