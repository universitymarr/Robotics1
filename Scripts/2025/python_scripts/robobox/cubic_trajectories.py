import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from .cubic_poly_dnorm_coeffs import cubic_poly_dnorm_coeffs

def cubic_trajectories(joint_params: list, profile: str='position'):
    """
    Displays the cubic trajectories for the provided joints
    
    @param joint_params: The joints parameters:  (qin, qfin, vin, vfin, T).
    @param profile: The profile displayed: 'position', 'velocity', 'acceleration'
    """
    tau = np.linspace(0, 1, 100)  # Time normalization

    if profile == 'position':
        plt.figure(figsize=(10, 6))
        plt.title("Cubic joint trajectories")
        plt.xlabel("Normalized Time (τ)")
        plt.ylabel("Position")
        
        for i, params in enumerate(joint_params):
            qin, qfin, vin, vfin, T = params
            q_tau, _ = cubic_poly_dnorm_coeffs(qin, qfin, vin, vfin, T)
            
            # Numerical conversion
            q_tau_func = sp.lambdify(sp.symbols('tau'), q_tau, 'numpy')
            q_values = q_tau_func(tau)

            plt.plot(tau, q_values, label=f"Joint {i+1}")
        plt.grid()

    elif profile == "velocity":
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
            
        ax1.set_title("Cubic joint velocities")
        ax1.set_xlabel("Normalized Time (τ)")
        ax1.set_ylabel("Velocity")
        
        ax2.set_title("Norm of cubic velocity profiles")
        ax2.set_xlabel("Normalized Time (τ)")
        ax2.set_ylabel("Velocity")

        velocities = []

        for i, params in enumerate(joint_params):
            qin, qfin, vin, vfin, T = params
            q_tau, _ = cubic_poly_dnorm_coeffs(qin, qfin, vin, vfin, T)
    
            q_tau_dot = sp.diff(q_tau)
            
            q_tau_dot_func = sp.lambdify(sp.symbols('tau'), q_tau_dot, 'numpy')
            v_values = q_tau_dot_func(tau)
            
            ax1.plot(tau, v_values, label=f"Joint {i+1}")
            velocities.append(v_values)

        norm_values = np.sqrt(np.sum(np.array(velocities)**2, axis=0))

        ax2.plot(tau, norm_values, label="Norm of velocity", color='m')

        ax1.legend()
        ax2.legend()
        ax1.grid()
        ax2.grid()

    elif profile == "acceleration":
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
            
        ax1.set_title("Cubic joint accelerations")
        ax1.set_xlabel("Normalized Time (τ)")
        ax1.set_ylabel("Acceleration")
        
        ax2.set_title("Norm of cubic acceleration profiles")
        ax2.set_xlabel("Normalized Time (τ)")
        ax2.set_ylabel("Acceleration")

        accelerations = []

        for i, params in enumerate(joint_params):
            qin, qfin, vin, vfin, T = params
            q_tau, _ = cubic_poly_dnorm_coeffs(qin, qfin, vin, vfin, T)
    
            q_tau_dot = sp.diff(q_tau)
            q_tau_ddot = sp.diff(q_tau_dot)

            
            q_tau_ddot_func = sp.lambdify(sp.symbols('tau'), q_tau_ddot, 'numpy')
            a_values = q_tau_ddot_func(tau)
            
            ax1.plot(tau, a_values, label=f"Joint {i+1}")
            accelerations.append(a_values)

        norm_values = np.sqrt(np.sum(np.array(accelerations)**2, axis=0))

        ax2.plot(tau, norm_values, label="Norm of acceleration", color='m')

        ax1.legend()
        ax2.legend()
        ax1.grid()
        ax2.grid()
    
    else: 
        raise ValueError("Wrong profile arguments. profile must be 'position', 'velocity' or 'acceleration'.")

    plt.legend()

    plt.show()

