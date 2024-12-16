%% -------------------- Exercise 1 --------------------

% Global variables
syms q1 q2 q3 L M N
n_joints = 3;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [ 0      L    0     q1   ;
             pi/2  0    M   q2 ;
             0  N    0    q3    ;
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
%% Linear Jacobian and analysis
J = DH_to_JL(DH_table, [q1, q2, q3]);

disp("-------------------------------------------------------------------")
% Now if I want to find the singularities, I check the solutions for the determinant of the Jacobian
% Then evaluate the Jacobian with the singular values

% Calculate the determinant of the Jacobian
det_J = simplify(det(J));

% Display the solutions and conditions
[sol_q1, sol_q2, sol_q3] = solve(det_J == 0, [q1, q2, q3]);
disp('All possible solutions for the symbols that make the determinant zero:');
disp('- First configuration: ');
disp([sol_q1(1), sol_q2(1), sol_q3(1)]);
disp('- Second configuration: ');
disp([sol_q1(2), sol_q2(2), sol_q3(2)]);

% Evaluate the rank of the Jacobian at the singular configurations
disp('Rank of the Jacobian at the first singular configuration:');
J_first = subs(J, [q1, q2, q3], [sol_q1(1), sol_q2(1), sol_q3(1)]);
disp(rank(J_first));

disp('Rank of the Jacobian at the second singular configuration:');
J_second = subs(J, [q1, q2, q3], [sol_q1(2), sol_q2(2), sol_q3(2)]);
disp(rank(J_second));
disp("-------------------------------------------------------------------")

%% -------------------- Exercise 2 --------------------
syms q1(t) q2(t) % Define q1 and q2 as functions of time
q1_sym = sym('q1');
q2_sym = sym('q2');

% Define the position vector p using symbolic variables
p = [q2_sym * cos(q1_sym); 
     q2_sym * sin(q1_sym)];
q = [q1; q2]; % Joint variable vector
q_sym = [q1_sym; q2_sym]; % Joint variable vector (symbolic)

% Compute the Jacobian matrix ----> J(q)
J_q_sym = jacobian(p, q_sym); % Compute the Jacobian matrix
disp("Jacobian of the position vector p:");
disp(J_q_sym);

% We need this to substitute the symbolic variables with the functions of time
J_q = subs(J_q_sym, [q1_sym, q2_sym], [q1(t), q2(t)]);

% Compute the Jacobian matrix w.r.t time ----> J(q)_dot
J_q_dot = diff(J_q, t);
J_q_dot = simplify(J_q_dot);

% Display the time derivative of the Jacobian matrix
disp('Time derivative of the Jacobian matrix J(q)_dot:');
disp(J_q_dot);
% --- If we want J(q)_dot * q_dot, then simplify(J_q_dot * diff(q, t))

% Now, we should isolate q_dot_dot from the equation J(q) * q_dot_dot + J(q)_dot * q_dot = 0
q_dot_dot = simplify(-inv(J_q) * J_q_dot * diff(q, t));
disp("-------------------------------------------------------------------")

%% -------------------- Exercise 3 --------------------
% General formula: tau = J(q)^T * F
syms t1 t2; % Define the symbolic variables t1 and t2
tau = [t1; t2];

% tau = J(q)^T * F
transp_J = transpose(J_q_sym); % Compute the torque vector T

% F = -inv(J(q)^T) * tau
force = simplify(-inv(transp_J)) * tau; % Compute the force vector F

% Substitute the symbolic variables with the actual values
force = subs(force, [q1_sym, q2_sym], [pi/3, 1.5]);         % Substitute the symbolic variables with the actual values
force_plus_plus = eval(subs(force, [t1, t2], [10, 5]));     % Substitute the symbolic variables with the actual values
force_plus_minus = eval(subs(force, [t1, t2], [10, -5]));   % Substitute the symbolic variables with the actual values
force_minus_plus = eval(subs(force, [t1, t2], [-10, 5]));   % Substitute the symbolic variables with the actual values
force_minus_minus = eval(subs(force, [t1, t2], [-10, -5])); % Substitute the symbolic variables with the actual values

% Display the results
disp('Force vector F for t1 = 10 and t2 = 5:');
disp(force_plus_plus);
disp('Force vector F for t1 = 10 and t2 = -5:');
disp(force_plus_minus);
disp('Force vector F for t1 = -10 and t2 = 5:');
disp(force_minus_plus);
disp('Force vector F for t1 = -10 and t2 = -5:');
disp(force_minus_minus);
disp("-------------------------------------------------------------------")

%% -------------------- Exercise 4 --------------------
% q_dot = J(q)^-1 * (p_dot_desired + Kp * (p_desired - p))
syms q1 q2 p1 p2 vd K
variables = [q1, q2]; % Define the joint variables

% Define the desired position vector p_desired
p = [cos(q1) + cos(q1 + q2);
     sin(q1) + sin(q1 + q2)];

J = jacobian(p, variables); % Compute the Jacobian matrix
J_inv = simplify(inv(J));   % Compute the inverse of the Jacobian matrix

p_desired = p1 + vd * (p2 - p1);    % Define the desired position vector p_desired
p_dot_desired = vd * (p2 - p1);     % diff(p_desired, t); % Compute the desired velocity vector p_dot_desired

% Retrieve the inverse kinematics of a 2R planar robot. We know the values of p_0 and L,
% so directly substitute them into the inverse_kinematic_2R_planar function
L = 1.0;
px = 0.5;
py = 0.5;
angles = inverse_kinematics_2R_planar(L, L, 0.5, 0.5, 'pos');
disp('Angles of the first and second joint:');
disp(angles);

% Now plug everything into the formula q_dot defined at the beginning
vd = 0.5;                           % Define the desired velocity vd
p1 = [1; 0.5];                      % Define the initial position p1
p2 = [1; 1.5];                      % Define the final position p2

J_inv_subs = eval(subs(J_inv, [q1, q2], angles));
p_dot_desired = eval(p_dot_desired);
p_desired = eval(p_desired);
p_q = eval(subs(p, [q1, q2], angles));

q_dot = J_inv_subs * (p_dot_desired + (p_desired - p_q));

disp('Joint velocities q_dot for the given values:');
disp(q_dot);
disp("-------------------------------------------------------------------")
