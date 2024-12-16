% -------------------- Exercise 1 --------------------

% Global variables
syms q1 q2 q3
syms a2 a1 a3 d3 d2 L
n_joints = 3;
variables = [q1, q2, q3];

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [   pi/2,  -a1,  q1,  0;
                  0, a2,   q2,  -pi/2;
                  0,  a3,  0, q3;
            ];

DH = DHmatrix(alpha, a, d, theta);

A = cell(1,n_joints);

% Compute each transformation matrix
% -- Remember:
%   A{1} = 0^A_1
%   A{2} = 1^A_2
%   A{3} = 2^A_3
%   A{4} = 3^A_4
%   ...
%   A{i} = i-1^A_i
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

% 2nd and 3rd bullet points

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

% We need to extract the rotation matrix 3^R_G. In this case, this is not computable on MATLAB
% since we do not have the End-Effector Frame. We can only maximum extract the rotation matrix 2^R_3
% with the following code:
% RF2_R_RF3 = A{3};
% RF2_R_RF3 = RF2_R_RF3(1:3, 1:3);
% For this reason, we simply look at the axis of frame 3 w.r.t to the EE-frame and compute the rotation matrix manually.
disp("-------------------------------------------------------------------")

% 4th bullet point
JL_time = simplify(jacobian(p, variables));

% Compute the Geometric Jacobian
prismatic_joints = [1, 2];
JL_geometric = sym(zeros(3, n_joints));

% Calculate z vectors
z = sym(zeros(3, n_joints));
z(:, 1) = [0; 0; 1];  % z0
T_cumulative = eye(4);

for i = 2:n_joints
    T_cumulative = T_cumulative * A{i-1};
    z(:, i) = T_cumulative(1:3, 3);
end

% Calculate p vectors (position of end-effector relative to each frame)
p_E = T(1:3, 4);  % End-effector position
p = sym(zeros(3, n_joints));
T_cumulative = eye(4);

for i = 1:n_joints
    p(:, i) = p_E - T_cumulative(1:3, 4);
    if i < n_joints
        T_cumulative = T_cumulative * A{i};
    end
end

% Compute the Jacobian
for i = 1:n_joints
    % Check if i is in the prismatic joints list
    if ismember(i, prismatic_joints)
        JL_geometric(:, i) = z(:, i);
    else
        JL_geometric(:, i) = simplify(cross(z(:, i), p(:, i)));
    end
end

JL_geometric = simplify(JL_geometric);

% 5th bullet point
jacobian_analysis(JL_time, variables)
JL_single = subs(JL_time, {q1, q2, q3}, {0, 0, pi/2});

% Directions/Velocities accessible by the robot gripper
accessible_directions = simplify(orth(sym(JL_single)));
disp('Accessible directions at the singular configuration:');
disp(accessible_directions);

% Directions/Velocities not accessible by the robot gripper
inaccessible_directions = null(transpose(JL_single));
disp('Inaccessible directions at the singular configuration:');
disp(inaccessible_directions);

% %% -------------------- Exercise 2 --------------------
syms q1 q2 q3 M N L
syms delta real positive

% Input
q_in = [a1+a3, 0, 0]; % Initial joint angles
q_fin = [a1, -delta, 0];       % Final joint angles
T = 2;                      % Time
p_dot_0 = [0; 0; 0];        % End effector velocity at the beginning
p_dot_1 = [0; 0; 0];        % End effector velocity at the end

% Direct Kinematics
directK = [     a3*sin(q3) - a1;
                            -q2;
            q1 - a2 - a3*cos(q3)];

[q_tau, coefficients] = cubic_compute_joint_trajectories(q_in, q_fin, T, p_dot_0, p_dot_1, [], directK, true);