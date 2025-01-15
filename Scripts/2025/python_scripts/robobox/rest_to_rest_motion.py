import numpy as np
import matplotlib.pyplot as plt

def rest_to_rest_motion(total_time: float, acceleration_value: float, speed_value: float, acceleration_time: float):
    """
    Displays the motion profile of a robot rest-to-rest motion

    @param total_time: The total time of the motion profile (in s).
    @param acceleration_value: The acceleration magnitude during the acceleration phase (in distance_unit/s^2).
    @param speed_value: The constant speed during the coast phase (in units of distance_unit/s).
    @param acceleration_time: The time duration of the acceleration phase (in s).
    """
    # Coast time
    coast_time = total_time - 2*acceleration_time
    
    # Time vector
    t1 = np.linspace(0, acceleration_time, 100)
    t2 = np.linspace(acceleration_time, acceleration_time + coast_time, 100)
    t3 = np.linspace(acceleration_time + coast_time, total_time, 100)
    time = np.concatenate([t1, t2, t3])
    
    # Acceleration profile
    a1 = np.ones_like(t1) * acceleration_value
    a2 = np.zeros_like(t2)
    a3 = np.ones_like(t3) * -acceleration_value
    acceleration = np.concatenate([a1, a2, a3])
    
    # Velocity profile
    v1 = np.cumsum(a1)*(t1[1] - t1[0])  
    v2 = np.ones_like(t2)*speed_value
    v3 = np.cumsum(a3)*(t3[1] - t3[0]) + v2[-1]
    velocity = np.concatenate([v1, v2, v3])
    
    # Position profile
    x1 = np.cumsum(v1) * (t1[1] - t1[0]) 
    x2 = np.cumsum(v2) * (t2[1] - t2[0]) + x1[-1]
    x3 = np.cumsum(v3) * (t3[1] - t3[0]) + x2[-1]
    position = np.concatenate([x1, x2, x3])

    # Display
    fig, axis = plt.subplots(3, 1, figsize=(8, 8))

    axis[0].plot(time, position, 'r-')
    axis[0].set_title('Position profile')
    axis[0].set_xlabel('Time (s)')
    axis[0].set_ylabel('Position')
    axis[0].grid()

    # Plot velocity vs time
    axis[1].plot(time, velocity, 'g-')
    axis[1].set_title('Velocity profile')
    axis[1].set_xlabel('Time (s)')
    axis[1].set_ylabel('Velocity')
    axis[1].grid()

    # Plot acceleration vs time
    axis[2].plot(time, acceleration, 'b-')
    axis[2].set_title('Acceleration profile')
    axis[2].set_xlabel('Time (s)')
    axis[2].set_ylabel('Acceleration')
    axis[2].grid()
    
    plt.tight_layout()
    plt.show()


