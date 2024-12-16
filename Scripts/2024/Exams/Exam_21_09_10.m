%% -------------------- Exercise 1 --------------------

% Global variables
syms q1 q2 q3
syms L
n_joints = 3;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [ 0  q1  0  0;
             0  L  0  q2;
             0  L  0  q3;
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

px = p(1);
py = p(2);
angle = q2 + q3;

task = [px; py; angle];
variables = [q1, q2, q3];

J = jacobian(task, variables);

% Since it's not a square matrix, for computing the determinant, we use J.T * J
detJ = simplify(det(J));

disp('Determinant of the Jacobian:');
disp(detJ);

% For finding the singularities, we have to solve the equation det(J.) = 0
% We can use the solve function to find the singularities w.r.t q1, q2, q3
[q1_sol, q2_sol, q3_sol] = solve(detJ == 0, variables);
q1_sol = simplify(q1_sol);
q2_sol = simplify(q2_sol);
q3_sol = simplify(q3_sol);

disp('All possible solutions for the symbols that make the determinant zero:');
disp('- First configuration: ');
disp([q1_sol(1), q2_sol(1), q3_sol(1)]);

% Evaluate the rank of the Jacobian at the singular configurations
disp('Rank of the Jacobian at the first singular configuration:');
J_first = subs(J, variables, [q1_sol(1), q2_sol(1), q3_sol(1)]);
disp(rank(J_first));

disp("-------------------------------------------------------------------")

% take sing configuration
J_single = subs(J, q2, pi/2);
J_single = simplify(J_single);

jacobian_analysis(J_single, variables);

% Task Velocity
r_dot = [1; 0; 0];
q_dot = simplify(pinv(J_single) * r_dot);
disp('Task velocity:');
disp(q_dot);

% Unfeasible task velocity
r_dot_2 = null(transpose(J_single));
disp('Unfeasible task velocity:');
disp(r_dot_2);

% Force | τ = J^T(q) * F
% Find a force F0 that genereates a torque τ = 0
% It means that F0 belongs to the null space of J^T_single
F0 = null(transpose(J_single));
disp('Force F0 that generates a torque τ = 0:');
disp(F0);

% Inverse Kinematics
L = 0.5;
x = 0.3;
y = 0.7;
alpha = pi/3;
[angles] = inverse_kinematics_PRR_planar(L, x, y, alpha, 'pos');

%% -------------------- Exercise 2 --------------------
L = 0.5;
x = 2*L;
y = 0;
alpha = pi/4;
[angles_initial] = inverse_kinematics_PRR_planar(L, x, y, alpha, 'pos');

alpha = -pi/4;
[angles_final] = inverse_kinematics_PRR_planar(L, x, y, alpha, 'pos');

% directK = [p(1), p(2), angle];
% [q_tau, coefficients] = cubic_compute_joint_trajectories(angles_initial, angles_final, 1, 0, 0, L, directK, true);

clear all;
syms t real
% Define constants
L = 0.5;
T = 1;
num_joints = 3;

% Define alpha as a function of t
alpha = pi/4 - pi/2 * (3 * (t/T)^2 - 2 * (t/T)^3);

q_tau = cell(1, 3);
q_tau{1} = 2*L * (1 - cos(alpha));
q_tau{2} = -alpha;
q_tau{3} = 2*alpha;

% E-E
pd_tau = cell(1, 3);
pd_tau{1} = 0; % x
pd_tau{2} = 1; % y
pd_tau{3} = alpha; % alpha

% Position
figure;
hold on;    
for i = 1:num_joints
    fplot(q_tau{i}, [0, T]);
end
title('Joint Positions');
xlabel('Time (s)');
ylabel('Position (rad or m)');
legend(arrayfun(@(x) sprintf('q%d', x), 1:num_joints, 'UniformOutput', false));
hold off;

% End-effector position
figure;
hold on;
for i = 1:3
    fplot(pd_tau{i}, [0, T]);
end
title('Task Position');
xlabel('Time (s)');
ylabel('Position (m or rad)');
legend('px', 'py', 'alpha');
hold off;