function [q_out, guesses, cartesian_errors] = gradient_method(q_in, desired_point, f_r, initial_guess, alpha, max_iterations, max_cartesian_error, min_joint_increment, max_closeness_singularity)
% [q_out, guesses, cartesian_errors] = gradient(q_in, desired_point, f_r, 
%  initial_guess, alpha, max_iterations, max_cartesian_error, 
%  min_joint_increment, max_closeness_singularity) takes as inputs:
%   -q_in: The variables of the joints, e.g. [q1 q2 q3 q4]
%   -desired_point: the configuration we wish to reach
%   -f_r: The mapping from joints to points
%   -initial_guess: Initial configuration of joints
%   -alpha: The "learning rate" of the algorithm
%   -max_iterations: The maximum number of iterations we can perform
%   -max_cartesian_error: The level of precision we wish to reach
%   -min_joint_increment: The minimum level of increment of accuracy 
%                          between successive iterations.
%   -max_closeness_singularity: How close to a singularity we can get
% and outputs:
%   -q_out: The best reached configuration
%   -guesses: The history of tested configurations
%   -cartesian_errors: The history of errors

    J = jacobian(f_r, q_in);
    
    n_dof = length(q_in);
    n_config = length(desired_point);
    
    simple_case = n_config == n_dof;
    
    J_trs = J';
    
    guesses = zeros(max_iterations + 1, n_dof);
    cartesian_errors = zeros(1, max_iterations + 1);
    
    guess = initial_guess;
    
    for i = 1:max_iterations
        guesses(i, :) = guess;
        
        error = norm(desired_point - subs(f_r, q_in, guess));
        cartesian_errors(i) = error;
        
        if error <= max_cartesian_error
            fprintf("Finished at iteration %d\nBecause the error was lower than the specified amount.\n", i);
            break
        end
        
        if simple_case
            if abs(det(subs(J, q_in, guess))) <= max_closeness_singularity
                fprintf("Finished at iteration %d\nBecause we ended up too close to a singularity.\n", i);
                break
            end
        else
            [~, D, ~] = svd(subs(J, q_in, guess));
            if min(min(D)) >= D(1, 1) - max_closeness_singularity
                fprintf("Finished at iteration %d\nBecause we ended up too close to a singularity.\n", i);
                break
            end
        end
        
        p_k = gradient(eval(subs(f_r, q_in, guess)));
        while -p_k'*gradient(eval(subs(f_r, q_in, guess + alpha*p_k))) > -0.9*p_k'*gradient(eval(subs(f_r, q_in, guess)))%| subs(f_r, q_in, guess + alpha*desired_point) > subs(f_r, q_in, guess) + 1e-4*alpha*desired_point'*gradient(eval(subs(f_r, q_in, guess)))
            alpha = 0.99*alpha;
        end
        
        new_guess = guess + alpha * subs(J_trs, q_in, guess) * (desired_point - subs(f_r, q_in, guess));
        
        if norm(new_guess - guess) <= min_joint_increment
            fprintf("Finished at iteration %d\nBecause of a too little increase between iterations.\n", i);
            break
        end
        
        guess = eval(new_guess);
    end

    guesses = guesses(1:i, :);
    cartesian_errors = cartesian_errors(1:i);
    q_out = guess;
end


