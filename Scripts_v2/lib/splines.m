function [coeffs, v] = splines(tvals, qvals, v1, vn, norm)
    % If initial and final velocities need to be imposed,
    % this needs modifications (slide 13).

    % tau = t - t1 --> tau = (t - t1) / h

    if exist('norm', 'var')
        normalize = norm;
    else
        normalize = 1;
    end


    %% Calcs
    N = size(tvals, 2);
    t = sym ('t', [1, N], 'real');
    q = sym ('q', [1, N], 'real');
    v = sym ('v', [1, N], 'real');

    if ~exist('v1', 'var')
        v1 = 0;
        vn = 0;
    end

    v(1) = v1;
    v(N) = vn;

    h = t(2:N) - t(1:N-1);

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

    A, b

    for i = 1:N-1
        fprintf('Symbolic Cubic %d parameters:\n', i);

        a0 = q(i);
        if normalize == 1
            a1 = v(i) * h(i);
            a = inv([1, 1; 2/h(i), 3/h(i)]) * [q(i+1) - q(i) - v(i) * h(i); v(i+1) - v(i)];
        else
            a1 = v(i);
            a = inv([h(i)^2, h(i)^3; 2*h(i), 3*h(i)^2]) * [q(i+1) - q(i) - v(i) * h(i); v(i+1) - v(i)];
        end

        % For normalization use this one:
        a = simplify([a0; a1; a])
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
    v = eval([v1; v; vn]);


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
end
