function [time, position, velocity, acceleration] = reverse_restToRestMotion(totalTime, accelerationValue, speedValue, accelerationTime, initialPosition, finalPosition)
    % This function simulates the motion profile of an object going from rest to rest in the reverse direction
    % Use: when initialPosition > finalPosition
    %
    % Parameters:
    % - totalTime: Total time of the motion profile.
    % - accelerationValue: Magnitude of acceleration during the acceleration and deceleration phases.
    % - speedValue: Constant speed during the coasting phase.
    % - accelerationTime: Time duration of the acceleration and deceleration phases.
    % - initialPosition: Initial position of the object.
    % - finalPosition: Final position of the object.
    %
    % Outputs:
    %  1. Plot of position wrt time
    %  2. Plot of speed wrt time
    %  3. Plot of acceleration wrt time

    % Reverse acceleration and speed values for motion in the opposite direction
    accelerationValue = -accelerationValue;
    speedValue = -speedValue;

    % Calculate coast time
    coastTime = totalTime - 2 * accelerationTime;
    
    % Time vector
    t1 = linspace(0, accelerationTime, 100);
    t2 = linspace(accelerationTime, accelerationTime + coastTime, 100);
    t3 = linspace(accelerationTime + coastTime, totalTime, 100);
    time = [t1, t2, t3];
    
    % Acceleration profile
    a1 = ones(1, 100) * accelerationValue;
    a2 = zeros(1, 100);
    a3 = -ones(1, 100) * accelerationValue;
    acceleration = [a1, a2, a3];
    
    % Velocity profile
    v1 = cumtrapz(t1, a1);
    v2 = ones(1, 100) * speedValue;
    v3 = cumtrapz(t3, a3) + v2(end);
    velocity = [v1, v2, v3];
    
    % Position profile (updated integration with initial and final positions)
    x1 = initialPosition + cumtrapz(t1, v1);
    x2 = cumtrapz(t2, v2) + x1(end);
    x3 = cumtrapz(t3, v3) + x2(end) + finalPosition;
    position = [x1, x2, x3];
      
    % Plotting the profiles
    figure;
    subplot(3, 1, 1);
    plot(time, position, 'b.-');
    title('Position vs Time');
    xlabel('Time (s)');
    ylabel('Position');
    
    subplot(3, 1, 2);
    plot(time, velocity, 'g.-');
    title('Velocity vs Time');
    xlabel('Time (s)');
    ylabel('Velocity');
    
    subplot(3, 1, 3);
    plot(time, acceleration, 'r.-');
    title('Acceleration vs Time');
    xlabel('Time (s)');
    ylabel('Acceleration');
end
