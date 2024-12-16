%% -------------------- Exercise 1a --------------------

% Global variables
syms q1 q2 q3 d1 a3
n_joints = 3;
variables = [q1, q2, q3];

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [ pi/2,  0, d1,   q1;
             pi/2,  0, q2, pi/2;
                0, a3,  0,   q3;
            ];

DH = DHmatrix(alpha, a, d, theta);

A = cell(1, n_joints);

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

disp("-------------------------------------------------------------------")

%% -------------------- Exercise 1b --------------------
%% Linear Jacobian and analysis
J = DH_to_JL(DH_table, variables);

disp("-------------------------------------------------------------------")
% Now if I want to find the singularities, I check the solutions for the determinant of the Jacobian
% Then evaluate the Jacobian with the singular values

% Calculate the determinant of the Jacobian
det_J = simplify(det(J));

% Display the solutions and conditions
[sol_q1, sol_q2, sol_q3, ~, ~] = solve(det_J == 0, variables, 'ReturnConditions', true);

disp('All possible solutions for the symbols that make the determinant zero:');
disp('- First configuration: ');
disp([sol_q1(1), sol_q2(1), sol_q3(1)]);
disp('- Second configuration: ');
disp([sol_q1(2), sol_q2(2), sol_q3(2)]);

% ------ J_1 ------
%
% Evaluate the rank of the Jacobian at the singular configurations
disp('Rank of J_1 - Namely when q3 = 0:');
J_first = subs(J, q3, 0);
disp(rank(J_first));

% Compute the nullspace of J_1
%
% The nullspace of the Jacobian represents the joint velocities that result in no end-effector velocity,
% i.e., configurations where the manipulator is at a singularity.
% We can either calculate this globally or in a specific singular configuration (in this case q3 = 0).
%
% The nullspace vector calculated below indicates that the joint configuration involving these velocities 
% will not change the position of the end-effector. In other words, certain combinations of joint movements
% (specifically -a3 in this case) do not result in any Cartesian movement of the end-effector,
% indicating a loss of degree of freedom in motion.
nullspace_J = null(J_first); 
disp('Nullspace of J_1:');
disp(simplify(nullspace_J));

% In order to show Cartesian direction(s) where instantaneous mobility of P is lost at the singular configuration
% We need to compute the complementary space of the range of the Jacobian, which also is the
% null space of the transpose of the Jacobian. (R^T(J) = N(J^T))
% Also, this states the direction(s) along which no task velocity can be realized.
nullspace_J_transpose = null(J_first');
disp('Nullspace of the J^T_1:');
disp(simplify(nullspace_J_transpose));

% ------ J_2 ------
disp('Rank of J_2 - Namely when q2 = -a3 * sin(q3) | q3 = pi*k:');
k = 1;
J_second = subs(J, q2, -a3 * sin(q3));
disp(rank(J_second));

% Compute the nullspace of J_2
nullspace_J = null(J_second); 
disp('Nullspace of the J_2:');
disp(simplify(nullspace_J));

% Compute the nullspace of J^T_2
nullspace_J_transpose = null(J_second');
disp('Nullspace of the J^T_2:');
disp(simplify(nullspace_J_transpose));

disp("-------------------------------------------------------------------")

%% -------------------- Exercise 1c --------------------
p_in = [2 + 1/sqrt(2), 1/sqrt(2)]';
p_out = [3/sqrt(2), - 1/sqrt(2)]';
L1 = 2;
L2 = 1;

% Find the inverse kinematics
q_p_in = inverse_kinematics_2R_planar(L1, L2, p_in(1), p_in(2), 'pos');
q_p_out = inverse_kinematics_2R_planar(L1, L2, p_out(1), p_out(2), 'pos');