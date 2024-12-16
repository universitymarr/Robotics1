%% -------------------- Exercise 1 --------------------

% Global variables
syms q1 q2 q3 q4 L D
n_joints = 4;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [    0,   0,  0, q1;
             pi/2,  q2,  0,  0;
            -pi/2,   0, q3,  0;
                0,   L,  0, q4;
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

% 2nd bullet point - Compute maximum possible distance between the end-effector and the origin of the base frame
norm_P = simplify(norm(p(1:2)));

disp("Norm of the position vector p:")
disp(norm_P)

norm_P = subs(norm_P, [q2, q3], [D, D]);
disp("Norm of the position vector p:")
disp(norm_P)

%% -------------------- Exercise 3 --------------------
syms q1 q2 

% Variables
variables = [q1; q2];

% Data
l1 = 0.5;
l2 = 0.4;
p_des = [0.4; -0.3];
q_in = [0.3491; -2.0944];

% Direct Kinematics
p = [l1*cos(q1)+l2*cos(q1+q2);
     l1*sin(q1)+l2*sin(q1+q2)];

% Newton method parameters
max_iterations = 3;
max_cartesian_error = 0.0001;

[q_out, guesses, cartesian_errors] = newton_method(variables, p_des, p, q_in, max_iterations, max_cartesian_error, 0.000001, 0.000001);

% Evaluate P at last guess
p_out = subs(p, variables, q_out);

disp("----------------------")
disp("Final position:")
disp(eval(simplify(p_out)))

disp("Final error value:")
disp(eval(norm(p_des - p_out)))

% ---------------------- Exercise 4 --------------------
% Global variables
syms q1 q2 q3 q4  a4
n_joints = 4;
variables = [q1; q2; q3; q4];

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [ pi/2,   0,  0, q1;
             pi/2,   0,  0, q2;
            -pi/2,   0, q3,  0;
                0,  a4,  0, q4;
            ];

DH = DHmatrix(alpha, a, d, theta);

A = cell(1,n_joints);

for i = 1: n_joints
    alpha = DH_table(i,1);
    a = DH_table(i,2);
    d = DH_table(i,3);
    theta = DH_table(i,4);
    A{i} = subs(DH);
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

% 1st request: Compute the Jacobian matrix
JL = simplify(DH_to_JL(DH_table, variables));
JA = simplify(DH_to_JA(DH_table, 3));

J = [JL; JA];
disp("Jacobian matrix:")
disp(J)

% As stated by the professor, we must express the Jacobian Matrix in another frame, which in this case is RF1
A_1 = A{1};
R_1 = A_1(1:3, 1:3);
JL_Rotated = simplify(R_1.' * JL);
JA_Rotated = simplify(R_1.' * JA);

% Total Jacobian
J_Rotated = [JL_Rotated; JA_Rotated];
disp("Rotated Jacobian matrix:");
disp(J_Rotated);

% Build the joint velocities ˙q ∈ R4 to the six-dimensional twist vector composed by a velocity v = v4 ∈ R3
% of the origin of the last (end-effector) DH frame and by an angular velocity ω = ω4 ∈ R3 of the same frame.
%
% Hence, we must compute the following: (v w) = J * q_dot
%
% So we need q_dot
syms q1(t) q2(t) q3(t) q4(t)
q1_sym = sym('q1');
q2_sym = sym('q2');
q3_sym = sym('q3');
q4_sym = sym('q4');
q_sym = [q1_sym; q2_sym; q3_sym; q4_sym];
q_sym = subs(q_sym, [q1_sym, q2_sym, q3_sym, q4_sym], [q1(t), q2(t), q3(t), q4(t)]);

% Define q_dot
q_dot = diff(q_sym, t);

% Compute v and w
vw = J * q_dot;
disp("v and w:")
disp(simplify(vw))

% τ =J^T(q) * (f µ)
% where f is the force vector and µ is the torque vector

% Singular configurations of the Jacobian
% Thanks to the inspection, we have seen that there are 2-submatrix in which we can compute when det == 0
% These submatrices are J13 and J19
syms q2 q3 q4
J13 = -q3;
J19 = sin(q2)*q3^2 + a4*cos(q2 + q4)*q3;

% Set up the equation and solve it
eqn = [ det(J13) == 0;
        det(J19) == 0];
[q2_sol, q3_sol, q4_sol, ~, ~] = solve(eqn, [q2, q3, q4], 'ReturnConditions', true);

disp("Solutions for the singular configurations:")
disp(['Solution for q2: ', char(q2_sol)])
disp(['Solution for q3: ', char(q3_sol)])
disp(['Solution for q4: ', char(q4_sol)])

% Compute the Jacobian at its singular configuration and check its rank (Should be < 4)
J_sing = subs(J_Rotated, q3, 0);
rank_J = rank(J_sing);
disp("Rank of the Singular Jacobian:")
disp(rank_J)

% E-E twist not realizable
disp("End-effector twist not realizable")
disp(null(J_Rotated.'))

% Basis of Joint velocities
disp("Basis of Joint velocities:")
disp(null(J_sing))

disp("We will now display the basis of N(J^T(q_sing)) back into the base frame:")

% Compute base frame
basis = null(J_sing.');

for i = 1:size(basis, 2)
    linear_part = basis(1:3, i);
    angular_part = R_1 * basis(4:6, i);
    twist = [linear_part; angular_part];
    disp("Basis " + i + ":")
    disp(twist)
end

% E-E wrenches for which the manipulator is statically balanced at q3 = 0,
% we need to determine a basis for the nullspace of the transpose of the geometric Jacobian.
% Which we already did at 237, so the answer is the same
% In fact, it's easy to see that if we take any basis and multiply it by the Jacobian, we will get 0
disp("J^T(q_sing) * basis:")
basis_1 = basis(:, 1);
disp(simplify(J_sing.' * basis_1))