%% -------------------- Exercise 1 --------------------

% Global variables
syms q1 q2 q3 q4
syms A B C D
n_joints = 4;
digits(5)

% alpha, a_i, d_i, beta_i
syms alpha a d beta
DH_table = [ pi/2, B,  A, q1;
                0, C,  0, q2;
             pi/2, D,  0, q3;
                0, 0, q4,  0;
            ];

DH = DHmatrix(alpha, a, d, beta);

A = cell(1,n_joints);

% Compute each transformation matrix
for i = 1: n_joints
    A{i} = subs(DH, {alpha, a, d, beta}, DH_table(i, :));
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

% Compute the Jacobian
variables = [q1, q2, q3, q4];
J = simplify(jacobian(p, variables));
disp('Jacobian:')
disp(J)

% We want to express the Jacobian in a rotated frame, for example in the 2nd frame
% We must extract the rotation matrix from the transformation matrix A(2)
J = simplify(subs(J, {B, D}, {0, 0}));

R_0_1 = A{1}(1:3, 1:3);
J_rotated = simplify(transpose(R_0_1) * J);

%jacobian_analysis(J_rotated)
J_rotated_sing = subs(J_rotated, C * cos(q2) + q4 * sin(q2 + q3), 0);

% For computing a feasible end-effector velocity, we need to compute the reduced Jacobian
% on a singular configuration
q_sing = [pi/2, pi/2, 0, 0];
J_reduced_q_singular = simplify(subs(J, variables, q_sing));

% Compute the range of the jacobian
range = simplify(orth(sym(J_reduced_q_singular)));

disp('Basis of the range:')
% Display the basis of the range
[~,m]=size(range);
for i=1:m
    [numerator, ~] = numden(range(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end

% Choose a feasible end-effector velocity
% For example, let's use the first basis vector of the range
v_star = [ 0;
          -C;
           0];

% Compute the pseudoinverse of J_reduced_2
J_pinv = simplify(pinv(J_reduced_q_singular));

% Compute q_dot_star
q_dot_star = simplify(J_pinv * v_star);

disp('q_dot_star:')
disp(q_dot_star)

%% -------------------- Exercise 2 --------------------
R1 = [0, -(sqrt(2)/2), sqrt(2)/2;
      1,  0, 0;
      0, sqrt(2)/2, sqrt(2)/2];

R_via = [sqrt(6)/4, sqrt(2)/4, -(sqrt(2)/2);
        -sqrt(6)/4, -(sqrt(2)/4), -(sqrt(2)/2);
            -1/2,       sqrt(3)/2, 0];

R2 = [sqrt(2)/2, 1/2, -1/2;
        0,  -(sqrt(2)/2), -(sqrt(2)/2);
        -(sqrt(2)/2), 1/2, -1/2];

T1 = 2.5;
T2 = 1;

% Compute the rotation matrix and the angle
sequence = char("ZYX");
[alpha_R1, beta_R1, gamma_R1] = euler_rotation_inverse(sequence, R1, "pos");
[alpha_R_via, beta_R_via, gamma_R_via] = euler_rotation_inverse(sequence, R_via, "pos");
[alpha_R2, beta_R2, gamma_R2] = euler_rotation_inverse(sequence, R2, "pos");

[q_alpha] = cubic_poly_double_norm_compute_coeff(alpha_R1, alpha_R_via, 0, 0, T1, true);
[q_beta] = cubic_poly_double_norm_compute_coeff(beta_R1, beta_R_via, 0, 0, T1, true);
[q_gamma] = cubic_poly_double_norm_compute_coeff(gamma_R1, gamma_R_via, 0, 0, T1, true);

[q_alpha_t2] = cubic_poly_double_norm_compute_coeff(alpha_R_via, alpha_R2, 0, 0, T2, true);
[q_beta_t2] = cubic_poly_double_norm_compute_coeff(beta_R_via, beta_R2, 0, 0, T2, true);
[q_gamma_t2] = cubic_poly_double_norm_compute_coeff(gamma_R_via, gamma_R2, 0, 0, T2, true);

syms tau t
quint_sub_alpha_t1 = subs(q_alpha, tau, (t/T1));
quint_sub_beta_t1 = subs(q_beta, tau, (t/T1));
quint_sub_gamma_t1 = subs(q_gamma, tau, (t/T1));

quint_sub_alpha_t2 = subs(q_alpha_t2, tau, ((t - T1)/T2));
quint_sub_beta_t2 = subs(q_beta_t2, tau, ((t - T1)/T2));
quint_sub_gamma_t2 = subs(q_gamma_t2, tau, ((t - T1)/T2));

% Alpha | Plot both segments at once from 0 to T1 and from T1 to T2
figure;
hold on;
grid on;
fplot(quint_sub_alpha_t1, [0, T1]);
fplot(quint_sub_alpha_t2, [T1, T1 + T2]);
title('Angle Alpha');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');

% Beta | Plot both segments at once from 0 to T1 and from T1 to T2
figure;
hold on;
grid on;
fplot(quint_sub_beta_t1, [0, T1]);
fplot(quint_sub_beta_t2, [T1, T1 + T2]);
title('Angle Beta');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');

% Gamma | Plot both segments at once from 0 to T1 and from T1 to T2
figure;
hold on;
grid on;
fplot(quint_sub_gamma_t1, [0, T1]);
fplot(quint_sub_gamma_t2, [T1, T1 + T2]);
title('Angle Gamma');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');