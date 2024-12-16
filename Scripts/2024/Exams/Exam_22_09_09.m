%% -------------------- Exercise 3 --------------------
syms q1 q2 q3 L

r = [q2 * cos(q1) + L * cos(q1 + q3);
     q2 * sin(q1) + L * sin(q1 + q3);
     q1 + q3];

variables = [q1, q2, q3];

% First bullet point:
% Find Jacobian by differentiating r with respect to q
J = simplify(jacobian(r, variables));

detJ = simplify(det(J));
[q1_sol, q2_sol, q3_sol, ~, ~] = solve(detJ == 0, variables, 'ReturnConditions', true);
disp('All possible solutions for the symbols that make the determinant zero:');
display([q1_sol, q2_sol, q3_sol]);
disp("--------------------")

% Second bullet point:
% When the robot is in a singularity, determine all possible task velocities r_dot
J_singular = subs(J, q2, 0);

% We know that r_dot = J * q_dot
syms q1(t) q2(t) q3(t)
q1_sym = sym('q1');
q2_sym = sym('q2');
q3_sym = sym('q3');
q_sym = [q1_sym; q2_sym; q3_sym];
q_sym = subs(q_sym, [q1_sym, q2_sym, q3_sym], [q1(t), q2(t), q3(t)]);

% Define q_dot
q_dot = diff(q_sym, t);

% Compute r_dot
r_dot = simplify(J_singular * q_dot);

disp("Task velocities r_dot:")
disp(r_dot)
disp("--------------------")

% Third bullet point:
% Calculate r_dot_dot as J(q) * q_dot_dot + J_dot(q) * q_dot
q_dot_dot = diff(q_dot, t);

% ------- Calculate J_dot(q)
% Define the position vector p using symbolic variables
r_sym = [q2_sym * cos(q1_sym) + L * cos(q1_sym + q3_sym);
        q2_sym * sin(q1_sym) + L * sin(q1_sym + q3_sym);
        q1_sym + q3_sym];
q_sym = [q1_sym; q2_sym; q3_sym];

% Compute the Jacobian matrix ----> J(q)
J_q_sym = jacobian(r_sym, q_sym); % Compute the Jacobian matrix
disp("Jacobian of the position vector p:");
disp(J_q_sym);

% We need this to substitute the symbolic variables with the functions of time
J_q = subs(J_q_sym, [q1_sym, q2_sym, q3_sym], [q1(t), q2(t), q3(t)]);

% Compute the Jacobian matrix w.r.t time ----> J(q)_dot
J_q_dot = diff(J_q, t);
J_q_dot = simplify(J_q_dot);
% -------

h = simplify(J_q_dot * q_dot);
final_r_dot_dot = simplify(J * q_dot_dot + h); % r_dot_dot = J(q) * q_dot_dot + J_dot(q) * q_dot

% Check determinant
if simplify(det(J_singular)) == 0
    disp('J(q) is singular. Infinitely many solutions exist.')
    % Find the null space
    null_space = simplify(null(J_singular));
    disp('Basis for the null space (possible q_dot_dot):')
    disp(null_space)
else
    disp('J(q) is non-singular. I can compute the inverse and apply q_dot_dot = 0 * J^(-1) = 0')
end
disp("--------------------")

% Fourth bullet point:
% Find again a joint acceleration q_dot_dot that realizes r_dot_dot = 0

% Substitution values
q1_val = pi/2;
q2_val = 1;
q3_val = 0;
q1_dot_val = 1;
q2_dot_val = -1;
q3_dot_val = -1;
L_val = 1;

% Compute the Jacobian
J = subs(J, variables, [q1_val, q2_val, q3_val]);
J = subs(J, L, L_val);

% Compute the h function by substituting the symbolic variables with the actual values
h_subbed = subs(h, L, L_val); % L
h_subbed = subs(h_subbed, {diff(q1(t), t), diff(q2(t), t), diff(q3(t), t)}, {q1_dot_val, q2_dot_val, q3_dot_val}); % q_dot
h_subbed = subs(h_subbed, {q1, q2, q3}, {q1_val, q2_val, q3_val}); % q

q_dot_dot = simplify((-inv(J)) * h_subbed); % q_dot_dot = -J^(-1) * h
disp("Joint acceleration q_dot_dot:")
disp(q_dot_dot)

% -------------------- Exercise 4 --------------------
qi = pi/2;
qi_dot = 1.5;
A = 4;
Td = qi_dot / A;

qd = qi + qi_dot * Td + (-A * Td^2)/2;
disp("Qd:")
disp(qd)