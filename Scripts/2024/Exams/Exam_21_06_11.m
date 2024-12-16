%% -------------------- Exercise 1 --------------------
digits(5)

% Global variables
syms q1 q2 q3 q4 q5 q6 q7
n_joints = 6;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [ -pi/2, 0, q1, 0;
             -pi/2, 0, q2, 0;
                 0, 0, q3, 0;
              pi/2, 0,  0, q4;
              pi/2, 0,  0, q5;
                 0, 0,  0, q6;
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
syms q1 q2 q3 L
p = [q2 * cos(q1) + L * cos(q1 + q3);
     q2 * sin(q1) + L * sin(q1 + q3);
     q1 + q3];
variables = [q1, q2, q3];

J = simplify(jacobian(p, variables)); % Jacobian matrix

% -- Exercise about Joint Velocity -- 
% Variables
v = [0; -2.5; 0]; % Vertical speed = Only y-axis is moving
L_val = 0.6;
px = 2;
py = 0.4;
[angles] = inverse_kinematics_RPR_planar(L_val, px, py, -pi/2, "pos");

% Evaluation | p˙ = J(q)q˙
J_val = eval(subs(J, [q1, q2, q3, L], [angles(1), angles(2), angles(3), L_val]));
q_dot_val = inv(J_val) * v;
disp("q_dot:")
disp(q_dot_val)

% -- Exercise about Force --
force = 15;
torque = 6;
F = [force; 0; torque];

% τ = -J^T(q) * F (Because it needs to resist the force)
tau = -transpose(J_val) * F;
disp("τ:")
disp(tau)

%% -------------------- Exercise 3 --------------------
syms A B s(t) % Define s and r as functions of time

% Inputs
Ra = [0, 1, 0;
      1, 0, 0;
      0, 0, -1];
Rb = [-1/sqrt(2),0,1/sqrt(2);
      0,  -1,  0;
      1/sqrt(2), 0, 1/sqrt(2)];

T = 2.5;

s_sym = sym('s');
s_sym_subs = subs(s_sym, sym('s'), s(t));

% Compute the position, velocity, and acceleration
p_s = A + (B - A) * s_sym_subs;
p_s_dot = diff(p_s, t);
p_s_dot_dot = diff(p_s_dot, t);

% Compute the rotation matrix and the angle
A_R_b = transpose(Ra) * Rb;
[r, theta] = axis_angle_rotation_inverse(A_R_b, 'pos');

% Angular timing law
theta_s = vpa(theta) * s_sym_subs;
theta_s_dot = diff(theta_s, t) * r;
theta_s_dot_dot = diff(theta_s_dot, t);

% Quintic timing law double normalized [0, 1] with rest-to-rest
syms tau t
tau_expr = t / T;
quint = 6 * tau^5 - 15 * tau^4 + 10 * tau^3;

quint_sub = subs(quint, tau, tau_expr);
quint_sub_dot = diff(quint_sub, t);
quint_sub_dot_dot = diff(quint_sub_dot, t);

syms t real

% Position
figure;
hold on;
grid on;
fplot(quint_sub, [0, T]);
title('Timing law');
xlabel('Time (s)');
ylabel('Position (rad)');
hold off;

% Velocity
figure;
hold on;
grid on;
fplot(quint_sub_dot, [0, T]);
title('Pseudo-Velocities');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
hold off;

% Acceleration
figure;
hold on;
grid on;
fplot(quint_sub_dot_dot, [0, T]);
title('Pseudo-Accelerations');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
hold off;

% Functions for finding the maxima
s_func = matlabFunction(quint_sub, 'Vars', t);
v_func = matlabFunction(quint_sub_dot, 'Vars', t);
a_func = matlabFunction(quint_sub_dot_dot, 'Vars', t);

% For position (maximum occurs at t = T)
[~, ~] = fminbnd(@(t) -s_func(t), 0, T);
s_max = s_func(T);  % The maximum is always at t = T for this profile

% For velocity (maximum occurs at t = T/2)
[~, v_max] = fminbnd(@(t) -v_func(t), 0, T);
v_max = -v_max;  % fminbnd finds minimum, so negate for maximum

% For acceleration (two maxima, one positive and one negative)
[~, a_max_pos] = fminbnd(@(t) -a_func(t), 0, T/2);
[~, a_max_neg] = fminbnd(@(t) a_func(t), T/2, T);
a_max_pos = -a_max_pos;  % fminbnd finds minimum, so negate for maximum
a_max_neg = a_func(a_max_neg);
a_max_abs = max(abs([a_max_pos, a_max_neg]));

fprintf('Maximum position: %f at t = %f\n', s_max, T);
fprintf('Maximum velocity: %f at t = %f\n', v_max, T/2);
fprintf('Maximum absolute acceleration: %f\n', a_max_abs);
fprintf('Positive acceleration peak: %f\n', a_max_pos);
fprintf('Negative acceleration peak: %f\n', a_max_neg);

% Now we must do the same for p_s / p_s_dot / p_s_dot_dot
% We will use the same timing law as before
A_eval = [1; 1; 1];
B_eval = [-1; 5; 0];

p_s_eval = subs(p_s, {A, B}, {A_eval, B_eval});
p_s_eval = subs(p_s_eval, s(t), quint_sub);

p_s_dot_eval = subs(p_s_dot, {A, B}, {A_eval, B_eval});
p_s_dot_eval = subs(p_s_dot_eval, diff(s(t), t), quint_sub_dot);

p_s_dot_dot_eval = subs(p_s_dot_dot, {A, B}, {A_eval, B_eval});
p_s_dot_dot_eval = subs(p_s_dot_dot_eval, diff(s(t), t, t), quint_sub_dot_dot);

% Position
figure;
hold on;
grid on;
fplot(p_s_eval, [0, T]);
title('Position');
xlabel('Time (s)');
ylabel('Position (rad)');
hold off;

% Velocity
figure;
hold on;
grid on;
fplot(p_s_dot_eval, [0, T]);
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
hold off;

% Acceleration
figure;
hold on;
grid on;
fplot(p_s_dot_dot_eval, [0, T]);
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
hold off;

% Now we must do the same for theta_s / theta_s_dot / theta_s_dot_dot
% We will use the same timing law as before
theta_eval = subs(theta_s, s(t), quint_sub);
theta_dot_eval = subs(theta_s_dot, diff(s(t), t), quint_sub_dot);
theta_dot_dot_eval = subs(theta_s_dot_dot, diff(s(t), t, t), quint_sub_dot_dot);

% Position
figure;
hold on;
grid on;
fplot(theta_eval, [0, T]);
title('Angular Position');
xlabel('Time (s)');
ylabel('Position (rad)');
hold off;

% Velocity
figure;
hold on;
grid on;
fplot(theta_dot_eval, [0, T]);
title('Angular Velocity');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
hold off;

% Acceleration
figure;
hold on;
grid on;
fplot(theta_dot_dot_eval, [0, T]);
title('Angular Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
hold off;
