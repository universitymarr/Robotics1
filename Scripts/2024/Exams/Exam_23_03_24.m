%% -------------------- Exercise 1 --------------------

% Global variables
syms q1 q2 q3 q4 q5 q6 a5 a6
n_joints = 6;
variables = [q1, q2, q3, q4, q5, q6];

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [    0,  0, q1,  pi/2;
            -pi/2,  0, q2, -pi/2;
            -pi/2,  0, q3, -pi/2;
            -pi/2,  0,  0,    q4;
                0, a6,  0,    q5;
                0, a5,  0,    q6;
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

% 2nd bullet point: Compute the Jacobian matrix

JL = DH_to_JL(DH_table, variables);
JA = DH_to_JA(DH_table, [1, 2, 3]);
J = [JL; JA];

disp('Jacobian matrix:')
disp(J)

q0 = [1, 1, 1, -pi/2, -pi/2, -pi/2];

% Calculate Jacobian matrix by using true values for all q
J_q0 = subs(J, [q1, q2, q3, q4, q5, q6], q0);

% To check if this matrix is singular, we can check if the determinant is zero
% If the determinant is zero, the configuration is singular
detJ_q0 = det(J_q0);
disp('Determinant of Jacobian matrix at J(q0):')
disp(detJ_q0)

% 3rd bullet point: Substitute the q0 at position vector p
p_q0 = subs(p, [q1, q2, q3, q4, q5, q6], q0);
disp('Position vector p(q0):')
disp(p_q0)

% Desired linear and angular velocities of the end-effector
v_desired = [0.5, 2, -2].';   % Linear velocity [m/s]
omega_desired = [0, 3, 0].';  % Angular velocity [rad/s]

% Combine the desired linear and angular velocities into a 6x1 vector
v_combined = [v_desired; omega_desired];

% Solve for the joint velocities
pseudo_inverse_J = simplify(pinv(J_q0));
q_dot = pseudo_inverse_J * v_combined;

disp('Joint velocities (q_dot) to achieve the desired end-effector velocities:')
disp(q_dot)