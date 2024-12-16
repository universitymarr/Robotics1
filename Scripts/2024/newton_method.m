function [q_out, guesses, cartesian_errors] = newton_method(q_in, desired_point, f_r, initial_guess, max_iterations, max_cartesian_error, min_joint_increment, max_closeness_singularity)
% INPUTS:
%   - q_in                      : The variables of the joints, e.g. [q1 ;q2; q3 ;q4]
%   - desired_point             : the configuration we wish to reach [px; py; pz]
%   - f_r                       : The mapping from joints to points f(q)
%   - initial_guess             : Initial configuration of joints [qx; qy; qz]
%   - max_iterations            : The maximum number of iterations we can perform (if in doubt put high number)
%   - max_cartesian_error       : The level of precision we wish to reach (if not specified put 0.00000001)
%   - min_joint_increment       : The minimum level of increment of accuracy between successive iterations.
%   - max_closeness_singularity : How close to a singularity we can get (if not specified 0.000001)
%
% OUTPUTS:
%   - q_out             : The best reached configuration
%   - guesses           : The history of tested configurations
%   - cartesian_errors  : The history of errors

    J = jacobian(f_r, q_in);
    
    n_dof = length(q_in);
    n_config = length(desired_point);
    
    simple_case = n_config == n_dof;
    
    if n_config < n_dof
        disp("Redundant case, let's use the pseudo inverse");
        J_inv = simplify(pinv(J));
    else
        disp("Non-redundant case, let's use the inverse");
        J_inv = simplify(inv(J));
    end
    
    guesses = zeros(max_iterations + 1, n_dof);
    cartesian_errors = zeros(1, max_iterations + 1);
    
    guess = initial_guess;
    
    for i = 1:max_iterations
        guesses(i, :) = guess;
        
        error = norm(desired_point - subs(f_r, q_in, guess));
        cartesian_errors(i) = error;
        if error < max_cartesian_error
            fprintf("Finshed at iteration %d because the error was lower than the specified amount.\n", i-1);
            break
        end
        
        if simple_case
            if abs(det(subs(J, q_in, guess))) <= max_closeness_singularity
                fprintf("Finished at iteration %d because too close to a singularity.\n", i-1);
                break
            end
        else
            [~, D, ~] = svd(subs(J, q_in, guess));
            if min(min(D)) >= D(1, 1) - max_closeness_singularity
                fprintf("Finished at iteration %d because too close to a singularity.\n", i-1);
                break
            end
        end
        
        new_guess = guess + subs(J_inv, q_in, guess) * (desired_point - subs(f_r, q_in, guess));
        
        if norm(new_guess - guess) <= min_joint_increment
            fprintf("Finished at iteration %d because of a too little joints values increment.\n", i-1);
            break
        end
        
        guess = eval(new_guess);
    end
    guesses = guesses(1:i, :);
    for m= 0:i-1
        fprintf("Guess %d: \n", m)
        disp(guesses(m+1,:))
    end
    cartesian_errors = cartesian_errors(1:i);
    disp("Cartesian errors")
    disp(cartesian_errors)
    q_out = guess;
end
