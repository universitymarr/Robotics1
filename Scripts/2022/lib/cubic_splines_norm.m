function [coeffs, v] = cubic_splines_norm(tvals, qvals, v1, vn, print_info)

    if length(tvals) ~= length(qvals)
        fprintf("arrays of positions and times must have same size! \n")
    end
    
    % tau = t - t1 --> tau = (t - t1) / h
    normalize = 1;
%    if exist('norm', 'var')
%        normalize = norm;
%    else
%        normalize = 1;
%    end


    %% Calcs
    N = size(tvals, 2);
    t = sym ('t', [1, N], 'real');
    q = sym ('q', [1, N], 'real');
    v = sym ('v', [1, N], 'real');

    v(1) = v1;
    v(N) = vn;

    %build array of time intervals
    %% h(1) = t2 - t1
    %% h(2) = t3 - t2 and so on ...
    h = t(2:N) - t(1:N-1);

    
    if print_info
        fprintf("We need %d cubic poly\n", N-1);
        fprintf("For each k-th cubic function:\n")
        fprintf("\tq_k(tau) = a_k3*tau^3 + a_k2*tau^2 + a_k1*tau + a_k0\n")
        fprintf("\ttau € [0, 1]; tau = t-t_k/h_k; h_k = t_k+1 - t_k\n")
    end
    
    % A is a N-2 x N-2 matrix
    % The same for all joints
    A = sym(zeros([N-2, N-2]));
    for i = 1:N-2
        if i-1 > 0
            A(i-1, i) = h(i-1);
        end
        A(i, i) = 2 * (h(i) + h(i+1));
        if i+1 <= N-2
            A(i+1, i) = h(i+2);
        end
    end

    % b changes for each joint
    % as it depends on q
    b = sym(zeros([N-2, 1]));
    for i = 1:N-2
        b(i) = 3 * (q(i+2) - q(i+1)) * h(i)/h(i+1) + 3 * (q(i+1) - q(i)) * h(i+1)/h(i);
    end
    b = b - [h(2) * v1; zeros([N-4,1]); h(N-2) * vn];

    if print_info
        fprintf("First at all we compute the intermediate velocities v2 ... vn-1\n");
        fprintf("To do that we write the following linear system Ax=b where x=[v2 ... vn-1]\n");
        display(A);
        display(b);
    end
    


    
    %% Numberical Results
    % Convert everything to numerical values
    A = subs(A, t, tvals);
    b = subs(b, t, tvals);
    b = subs(b, q, qvals);
    h = subs(h, t, tvals);

    v1 = subs(v1, q, qvals);
    v1 = subs(v1, t, tvals);
    vn = subs(vn, q, qvals);
    vn = subs(vn, t, tvals);

    v = inv(A) * b;
    
    if print_info
        fprintf("By replacing symbols with numbers we obtain:\n ");
        display(A);
        display(b);
        fprintf("With the result:\n");
        display(v);
    end
    
    v = eval([v1; v; vn]);
    
    if print_info
        fprintf("All the velocities at each knots are the following:\n")
        display(v);
    end

    % compute coefficients
    %{ 
    NOT NORMALIZED
    if print_info
        fprintf("Now we compute coefficients of each cubic function");
        fprintf("For each function to compute a2 and a3 we have to solve the following linear system (Ax=b)")
        syms h_i q_i q_j v_i v_j
        A__ = [h_i^2, h_i^3;
             2*h_i, 3*h_i^2];
        
        b__ = [q_j - q_i - v_i * h_i; 
                            v_j - v_i];
        display(A__);
        display(b__);
        fprintf("Where j = i+1 and h_i = t_j - t_i\n");
        fprintf("Others coefficients are instead immediately computable:");
        fprintf("a0 = q_i\na1 = v_i\n");
    end
    %}
    
    if print_info
        fprintf("Now we compute coefficients of each cubic function\n");
        fprintf("For each function i-th to compute a2 and a3 we have to solve the following linear system (Ax=b)\n")
        syms h_i q_i q_j v_i v_j
        A__ = [1, 1;
             2/h_i, 3/h_i];
        
        b__ = [q_j - q_i - v_i * h_i; 
                            v_j - v_i];
        display(A__);
        display(b__);
        fprintf("Where j = i+1 and h_i = t_j - t_i\n");
        fprintf("Others coefficients are instead immediately computable:\n");
        fprintf("a0 = q_i\na1 = v_i*h_i\n");
    end
        
    coeffs = [];
    for i = 1:N-1
        % fprintf('Numeric Cubic %d parameters:\n', i);
        a0 = qvals(i);
        
        if normalize == 1
            a1 = v(i) * h(i);
            a = inv([1, 1; 2/h(i), 3/h(i)]) * [qvals(i+1) - qvals(i) - v(i) * h(i); v(i+1) - v(i)];
        else
            a1 = v(i);
            a = inv([h(i)^2, h(i)^3; 2*h(i), 3*h(i)^2]) * [qvals(i+1) - qvals(i) - v(i) * h(i); v(i+1) - v(i)];
        end
        a = eval([a0, a1, a']);

        coeffs = [coeffs; a];
    end
    
    if print_info
        fprintf("\nCoefficients of all cubic functions -------------------\n")
        display(coeffs);
        for i=1:size(coeffs, 1)
            fprintf('Symbolic Cubic %d parameters:\n', i);
            for j=1:size(coeffs,2)
                fprintf('a%d = %f \n', j-1, coeffs(i,j)); 
            end
            fprintf("\n");
        end
        fprintf("-------------------------------------------------------")
    end

end