%% -------------------- Exercise 1 --------------------

% Global variables
syms q1 q2 q3 q4 q5 q6
syms d1 d4 d6 a2 a3 a5
n_joints = 6;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [ pi/2   0   d1   q1;
             0      a2   0   q2;
             pi/2  a3   0    q3;
             -pi/2   0   d4  q4;
             pi/2  a5    0   q5;
                0    0   d6  q6;
            ];

DH = DHmatrix(alpha, a, d, theta);

A = cell(1,n_joints);

% Compute each transformation matrix
for i = 1: n_joints
    A{i} = subs(DH, {alpha, a, d, theta}, DH_table(i, :));
end
T = eye(4);

disp("-------------------------------------------------------------------")
for i = 1 : n_joints
    % Display print statement
    disp(['0^A_' num2str(i) '(q_' num2str(i) '):']);
    
    % Perform the simplify operation
    T = simplify(T * A{i});
    
    % Display the simplified result
    disp(T);
end

% output world-endEff T matrix
disp("-------------------------------------------------------------------")
disp('W^T_E:')
disp(T)

% Remember. T is a matrix 4x4
% Output WE position vector | From 1st to 3rd row of 4th column
p = T(1:3, 4);
disp('Position vector p:')
disp(p)

% Output xN axis | From 1st to 3rd row of 1st column
n = T(1:3, 1);
disp('x-axis (n):')
disp(n)

% Output yN axis | From 1st to 3rd row of 2nd column
s = T(1:3, 2);
disp('y-axis (s):')
disp(s)

% Output zN axis | From 1st to 3rd row of 3rd column
a = T(1:3, 3);
disp('z-axis (a):')
disp(a)

disp("-------------------------------------------------------------------")

%% -------------------- Exercise 2 --------------------
syms s(t) r(t) % Define s and r as functions of time
s_sym = sym('s');
r_sym = sym('r');

p_s = r_sym * [cos(s_sym); sin(s_sym)];
p_s = subs(p_s, [s_sym, r_sym], [s(t), r(t)]);

% Derivative w.r.t time
p_s_dot = jacobian(p_s, s(t));
p_s_dot_dot = jacobian(p_s_dot, s(t));

% Display the results
disp('Velocity vector p_s_dot:')
disp(p_s_dot)
disp('Acceleration vector p_s_dot_dot:')
disp(p_s_dot_dot)

% ---------------
% Calculations for s_dot and s_dot_dot
syms tau(t) delta(t) T
tau_sym = sym('tau');
delta_sym = sym('delta');

% Define the s function and tau as t/T
s = delta_sym * (-2 * tau_sym^3 + 3 * tau_sym^2);
tau_expr = t / T;

% Compute the derivative of s w.r.t tau and tau w.r.t time
derivative_s_tau = jacobian(s, tau_sym);
derivative_tau_t = jacobian(tau_expr, t);

% Compute s_dot and s_dot_dot
s_dot = derivative_s_tau * derivative_tau_t;                % Derivative w.r.t time of s
s_dot_dot = jacobian(s_dot, tau_sym) * derivative_tau_t;    % Derivative w.r.t time of s_dot

% Display the results
disp('Velocity vector s_dot:')
disp(s_dot)
disp('Acceleration vector s_dot_dot:')
disp(s_dot_dot)

% ---------------
% Now I have to calculate the velocity and acceleration of p(s)
% p(s)_dot = p'(s) * s_dot
% p(s)_dot_dot = p'(s) * s_dot_dot + p''(s) * s_dot^2
% ---------------
p_dot = p_s_dot * s_dot;
p_dot_dot = p_s_dot * s_dot_dot + p_s_dot_dot * s_dot^2;
norm_p_dot_dot = norm(p_dot_dot);

%% -------------------- Exercise 3 --------------------

% Geometric Jacobian = J(q) = [JL; JA]
% Global variables
syms q1 q2 q3 q4
syms d1 d2 a1 a3 a4
n_joints = 4;
variables = [q1, q2, q3, q4];

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [   0  a1   d1  q1;
            pi/2   0   d2  q2;
               0  a3    0  q3;
               0  a4    0  q4;
             ];

DH = DHmatrix(alpha, a, d, theta);

A = cell(1,n_joints);

% Compute each transformation matrix
for i = 1: n_joints
    A{i} = subs(DH, {alpha, a, d, theta}, DH_table(i, :));
end

T = eye(4);

disp("-------------------------------------------------------------------")
for i = 1 : n_joints
    % Display print statement
    disp(['0^A_' num2str(i) '(q_' num2str(i) '):']);
    
    % Perform the simplify operation
    T = simplify(T * A{i});
    
    % Display the simplified result
    disp(T);
end

% output world-endEff T matrix
disp("-------------------------------------------------------------------")
disp('W^T_E:')
disp(T)

% Remember. T is a matrix 4x4
% Output WE position vector | From 1st to 3rd row of 4th column
p = T(1:3, 4);
disp('Position vector p:')
disp(p)

% Output xN axis | From 1st to 3rd row of 1st column
n = T(1:3, 1);
disp('x-axis (n):')
disp(n)

% Output yN axis | From 1st to 3rd row of 2nd column
s = T(1:3, 2);
disp('y-axis (s):')
disp(s)

% Output zN axis | From 1st to 3rd row of 3rd column
a = T(1:3, 3);
disp('z-axis (a):')
disp(a)

disp("-------------------------------------------------------------------")

% For the Geometry Jacobian, we calculate the jacobian w.r.t the task (forward kinematics)
JL = DH_to_JL(DH_table, variables);
JA = DH_to_Ja(DH_table, []);
J = [JL; JA];

% We want to express the Jacobian in a rotated frame, for example in the 2nd frame
% We must extract the rotation matrix from the transformation matrix A(2)
A_1_2 = A{1} * A{2};
R_1_2 = A_1_2(1:3, 1:3);
JL_Rotated = simplify(R_1_2.' * JL);
JA_Rotated = simplify(R_1_2.' * JA);

disp("Rotated JL w.r.t the 2nd frame:");
disp(JL_Rotated);
disp("Rotated JA w.r.t the 2nd frame:");
disp(JA_Rotated);

% Total Jacobian
J_Rotated = [JL_Rotated; JA_Rotated];

% Define the transformation matrix T
T = [1, 0, 0, 0;
     -1, 1, 0, 0;
     0, 0, 1, 0;
     0, 0, -1, 1];

J_new = simplify(J_Rotated * T);

J14 = J_new([2:3, 5:6], [1:3, 4]);  % Minor by deleting row 4 and row 1
J24 = J_new([1, 3, 5:6], [1:3, 4]); % Minor by deleting row 4 and row 2
J34 = J_new([1:2, 5:6], [1:3, 4]);  % Minor by deleting row 4 and row 3
J54 = J_new([1:3, 6], [1:3, 4]);    % Minor by deleting row 4 and row 5
J64 = J_new([1:3, 5], [1:3, 4]);    % Minor by deleting row 4 and row 6
eqn = [det(J14) == 0;
       det(J24) == 0;
       det(J34) == 0;
       det(J54) == 0;
       det(J64) == 0];

qsing = solve(eqn, [q2, q3], 'Real', true);

disp('Singular configurations:');
disp(qsing);

% Calculate the Jacobian when q = 0
J_q_0 = subs(J, [q1, q2, q3, q4], [0, 0, 0, 0]);

Va = [0, 3, -3, 0, 0, 1]; % Task velocity

% Calculate the joint velocities q_dot = J_q^-1 * Va
% In this case, we use the pseudo-inverse of J_q_0 because the Jacobian is not square
q_0_dot = simplify(pinv(J_q_0)) * Va.';

disp('q_0_dot:');
disp(q_0_dot);