% -------------------- Exercise 1 --------------------
syms gamma(t) beta(t) alpha(t) rx ry rz % Define all the symbolic variables
gamma_sym = sym('gamma');
beta_sym = sym('beta');
alpha_sym = sym('alpha');

Rx = [1,               0,                0;
      0,  cos(alpha_sym),  -sin(alpha_sym);
      0,  sin(alpha_sym),   cos(alpha_sym);
     ];

Ry = [ cos(beta_sym),   0,   sin(beta_sym);
                   0,   1,               0;
      -sin(beta_sym),   0,   cos(beta_sym);
     ];

Rz = [cos(gamma_sym),  -sin(gamma_sym),   0;
      sin(gamma_sym),   cos(gamma_sym),   0;
                   0,                0,   1;
     ];

skew_matrix = [  0,  -rz,   ry;
                rz,    0,  -rx;
               -ry,   rx,    0;
               ];

Ry_exercise = subs(Ry, beta_sym, gamma_sym);
Rz_exercise = subs(Rz, gamma_sym, beta_sym);
R = simplify(Ry_exercise * Rz_exercise * Rx);

disp('Ry(gamma) * Rz(beta) * Rx(alpha) = ');
disp(R);
disp("----------------------------------");

% Now, since we have to compute the S(w) = R_dot * R.T,
% we have to compute the derivative of R with respect to time
R_var = subs(R, [alpha_sym, beta_sym, gamma_sym], [alpha(t), beta(t), gamma(t)]);
R_dot = diff(R_var, t);
R_dot = simplify(R_dot);
disp('Time derivative of the rotation matrix R:');
disp(R_dot);
disp("----------------------------------");

% Compute the skew-symmetric matrix S(w) = R_dot * R.T
S_w = simplify(R_dot * simplify(transpose(R_var)));
disp('Skew-symmetric matrix S(w) = R_dot * R.T:');
disp(S_w);
disp("----------------------------------");

% We can now extract the w vector from the matrix, taking these elements:
% skew_matrix = [  0,  -rz,   ry;
%                 rz,    0,  -rx;
%                -ry,   rx,    0;
% ];
% wx = S_w(3, 2)
% wy = S_w(1, 3)
% wz = S_w(2, 1)
disp('Angular velocity vector w = [wx; wy; wz]:');
w = [simplify(S_w(3, 2)); simplify(S_w(1, 3)); simplify(S_w(2, 1));];
disp(w);
disp("----------------------------------");

% Substitute the time derivatives
syms alpha_dot beta_dot gamma_dot
w_subs = subs(w, {diff(alpha(t), t), diff(beta(t), t), diff(gamma(t), t)}, {alpha_dot, beta_dot, gamma_dot});

% Define the vector of time derivatives
phi_dot = [alpha_dot; beta_dot; gamma_dot];

% Solve for the coefficients of each time derivative
T_phi = equationsToMatrix(w_subs, phi_dot);

disp('T(phi) = ');
disp(simplify(T_phi));
disp('phi_dot =')
disp(simplify(phi_dot));
disp("----------------------------------");

% In order to find singularities of T_phi, we have to solve the equation det(T_phi) = 0
det_T_phi = simplify(det(T_phi));
disp('Determinant of T(phi) = ');
disp(det_T_phi);

% Solve the equation det(T_phi) = 0
[alpha_sol, beta_sol, gamma_sol, ~, ~] = solve(det_T_phi == 0, [alpha(t), beta(t), gamma(t)], 'ReturnConditions', true);
disp('Singularities of T(phi) = ');
disp("Alpha: ");
disp(alpha_sol);
disp("Beta: ");
disp(beta_sol);
disp("Gamma: ");
disp(gamma_sol);
disp("----------------------------------");

% Compute the T_phi at the singularity
T_phi_singular = subs(T_phi, beta(t), pi/2);
disp('T_phi at singularity:');
disp(simplify(T_phi_singular));

% Compute the null space of T_phi_singular transpose.
% For a matrix equation A*x = b: There is no solution x if and only if b is in the null space of A^T
% Since our matrix equation is in the form of T(φ) * φ̇ = ω
% this translates in: There is no solution φ̇  if and only if ω is in the null space of T(φ)^T
N = simplify(null(transpose(T_phi_singular)));
disp('Vectors ω that cannot be represented by any ˙φ:');
disp(N);

% T(φ) * φ̇ = 0 where φ̇ ≠ 0
% This means, always considering A*x = b, that the set of all vectors x that satisfy Ax = 0 for some matrix A is called the null space of A
% We need to find the null space of T(φ) at the singularity, which is exactly what null(T_phi_singular) gives us
null_space_T = null(T_phi_singular);
disp('Non-trivial ˙φ that result in ω = 0:');
disp(simplify(null_space_T));

% -------------------- Exercise 2 --------------------
clear all; % Clear the workspace
% syms L M N q1 q2 q3

% p = [L * cos(q1) + N * cos(q1 + q2) * cos(q3);
%      L * sin(q1) + N * sin(q1 + q2) * cos(q3);
%      M + N * sin(q3);];

% % Define the z-component of p and the equation to solve
% syms pz
% eq = pz == p(3);
% sol = isolate(eq, sin(q3)); % Solve for sin(q3)
% disp(sol);                  % Display the result

% s3 = (-M + pz)/N; % Compute the value of sin(q3)
% c3 = simplify(sqrt(1 - s3^2)); % Compute the value of cos(q3)

% q3 = atan2(s3, c3); % Compute the value of q3

% % Now we want to find q1 from the first 2 equations in p
% l1 = L;
% l2 = N * cos(q3);

% c2 = (p(1)^2 + p(2)^2 - l1^2 - l2^2) / (2 * l1 * l2);
% s2 = sqrt(1 - c2^2);

% q2 = atan2(s2, c2);

% % Now, in order to find s1 and s2, we can set up a system of equations in the form:
% % Ax = b where x = [c1; s1], b = [p(1); p(2)] and A = [...]
% A = [l1 + l2 * c2, -l2 * s2;
%      l2 * s2, l1 + l2 * c2;];
% b = [p(1); p(2);];

% % Solve the system of equations by solving the Determinant of A
% det_A = simplify(det(A));
% disp('Determinant of A:');
% disp(det_A);

% % Now find s1 and c1, we must:
% % - Expand the sin/cos sum in p(1) and p(2)
% % - Simplify the notation:
% %   - a = L + N * cos(q2) * cos(q3)
% %   - b = N * sin(q2) * cos(q3)
% % So that the system of equations becomes:
% % a * c1 - b * s1 = p(1)
% % b * c1 + a * s1 = p(2)
% %
% % In order to solve for s1 and c1, we can use the Cramer's rule
% % s1 = ...
% % c1 = ...
% s1 = simplify(simplify(p(2) * (l1 + l2*c2) - p(1) * l2 * s2)/det_A);
% c1 = simplify(simplify(p(1) * (l1 + l2*c2) + p(2) * l2 * s2)/det_A);

% % Compute the value of q1
% q1 = atan2(s1, c1);

% Compute the inverse kinematics with numerical data
L = 0.5;
M = 0.5;
N = 0.5;
p = [0.3; -0.3; 0.7];
q = inverse_kinematics_3R_spatial(L, M, N, p, 'neg');
disp('Inverse kinematics solution:');
disp(q);

%% -------------------- Exercise 3 --------------------
syms q1 q2 q3

% Variables
variables = [q1; q2; q3];

% Data
L = 0.5;
M = 0.5;
N = 0.5;
p_des = [0.3; -0.3; 0.7];
q_in = [-pi/4; pi/4; pi/4];

% Direct Kinematics
p = [L * cos(q1) + N * cos(q1 + q2) * cos(q3);
     L * sin(q1) + N * sin(q1 + q2) * cos(q3);
     M + N * sin(q3);];

% Newton method parameters
max_iterations = 6;
max_cartesian_error = 0.001;

[q_out, guesses, cartesian_errors] = newton_method(variables, p_des, p, q_in, max_iterations, max_cartesian_error, 0.000001, 0.000001);

% %% -------------------- Exercise 4 --------------------
syms q1 q2 q3 M N L

% Input
q_in = [-pi/4, pi/4, pi/4]; % Initial joint angles
q_fin = [0, 0, pi/4];       % Final joint angles
T = 2;                      % Time
p_dot_0 = [1; -1; 0];       % End effector velocity at the beginning
p_dot_1 = [0; 0; 0];        % End effector velocity at the end
a_in = 0;                   % Initial acceleration
a_f = 0;                    % Final acceleration
L_val = 0.5;
M_val = 0.5;
N_val = 0.5;

% Direct Kinematics
directK = [L*cos(q1)+N*cos(q1+q2)*cos(q3);
            L*sin(q1)+N*sin(q1+q2)*cos(q3);
            M+N*sin(q3)];

% [q_tau, coefficients] = quintic_compute_joint_trajectories(q_in, q_fin, T, p_dot_0, p_dot_1, a_in, a_f, [L_val, M_val, N_val], directK, true);

num_joints = length(q_in);
link_lengths = [L_val, M_val, N_val];

% Jacobian
joint_vars = sym('q', [1 num_joints]);
all_vars = symvar(directK);
non_joint_vars = setdiff(all_vars, joint_vars);

J = simplify(jacobian(directK, joint_vars));

% Substitute initial values for joints and keep other variables symbolic
J_initial = subs(J, [joint_vars, non_joint_vars], [q_in, link_lengths]);

% Check if J_initial is square. If yes, inverse, otherwise we must compute the pseudo-inverse
if size(J_initial, 1) == size(J_initial, 2)
    J_initial_inv = simplify(inv(J_initial));
else
    J_initial_inv = simplify(pinv(J_initial));
end
disp(J_initial_inv)

% Velocity of configurations
q_dot_0 = J_initial_inv * p_dot_0;
q_dot_T = J_initial_inv * p_dot_1;

[q,m] = quintic_poly_double_norm_compute_coeff(q_in(1), q_fin(1), q_dot_0(1), q_dot_T(1), a_in, a_f, T, true);