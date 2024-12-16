% -------------------- Exercise 1 --------------------

% Global variables
syms q1 q2 q3 q4 q5 q6 q7
syms d1 d3 d5 d7
n_joints = 7;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [  pi/2,  0,  d1,  q1;
             -pi/2,  0,   0,  q2;
              pi/2,  0,  d3,  q3;
              pi/2,  0,   0,  q4;
             -pi/2,  0,  d5,  q5;
             -pi/2,  0,   0,  q6;
                 0,  0,  d7,  q7;
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

% We need to extract the rotation matrix 7^R_E. In this case, this is not computable on MATLAB
% since we do not have the End-Effector Frame. We can only maximum extract the rotation matrix 6^R_7
% with the following code:
% RF6_R_RF7 = A{7};
% RF6_R_RF7 = RF6_R_RF7(1:3, 1:3);
% For this reason, we simply look at the axis of frame 7 w.r.t to the EE-frame and compute the rotation matrix manually.
disp("-------------------------------------------------------------------")

% -------------------- Exercise 2 --------------------
% Define the Euler angles in radians

syms alpha1 alpha2 alpha3 phi

% Create rotation matrices for each axis
Ry1 = [cos(alpha1), 0, sin(alpha1);
                 0, 1,           0;
      -sin(alpha1), 0, cos(alpha1)];

Rx2 = [1,          0,           0;
      0, cos(alpha2), -sin(alpha2);
      0, sin(alpha2),  cos(alpha2)];

Ry3 = [cos(alpha3), 0, sin(alpha3);
                 0, 1,           0;
      -sin(alpha3), 0, cos(alpha3)];

% Compute the final rotation matrix
R = Ry1 * Rx2 * Ry3;
R = double(eval(subs(R, {alpha1, alpha2, alpha3}, {pi/4, -pi/4, 2*pi/3}))); % Substitute the values

% Display the result
disp('Rotation matrix:');
disp(R);

R_F = [ 0, sin(phi),  cos(phi);
        0, cos(phi), -sin(phi);
       -1,         0,       0];
R_F = double(eval(subs(R_F, phi, pi/3))); % Substitute the value
disp('Rotation matrix R_F:');
disp(R_F);

% Calculate the relative rotation
R_rel = transpose(R) * R_F;
disp('Relative rotation matrix R_rel:');
disp(R_rel);

% Use the provided function to get axis and angle
[axis, angle] = axis_angle_rotation_inverse(R_rel, "pos");

% Display results
disp('Axis of rotation:');
disp(axis);
disp('Angle of rotation (in radians):');
disp(angle);

% We need to calculate the time needed to reach the desired orientation
% The angular velocity is given by the formula |ω| = θ / T
% Therefore, the time needed is given by T = θ / |ω|
omega = 1.1 * axis;
time = angle / norm(omega);

% Display the time needed
disp('Time needed to reach the desired orientation (in seconds):');
disp(time);

% -------------------- Exercise 3 --------------------
% Define the direct kinematics of a 3R planar robot with q1, q2, q3 as the joint angles and L as the link length
% ==================
% The total formula is:
% p¨ = Jp(q) q¨ + J˙p(q) q˙
% We can call J˙p(q) q˙ = n, so we have:
% q¨ = inv(Jp(q)) * (p¨ - n)
% ==================
clear all;
syms q1(t) q2(t) q3(t) px py pz % Define q1 and q2 as functions of time
syms q1_dot q2_dot q3_dot % Define symbolic variables for the derivatives of q1, q2, q3
q1_sym = sym('q1');
q2_sym = sym('q2');
q3_sym = sym('q3');

q = [q1; q2; q3];                  % Joint variable vector
q_sym = [q1_sym; q2_sym; q3_sym];  % Joint variable vector
q_dot = [q1_dot; q2_dot; q3_dot];  % Joint velocity vector

% Define the position vector p using symbolic variables
p = [cos(q1_sym) + cos(q1_sym + q2_sym) + cos(q1_sym + q2_sym + q3_sym);
     sin(q1_sym) + sin(q1_sym + q2_sym) + sin(q1_sym + q2_sym + q3_sym)];

% Compute the Jacobian matrix ----> J(q)
Jp = simplify(jacobian(p, q_sym)); % Compute the Jacobian matrix

% Since we want to keep the angular velocity constant, it means that its derivative is zero.
% So we inglobe the angular velocity in the Jacobian matrix
w = [1; 1; 1]; % Angular velocity vector
Jp = [Jp; w']; % Add the angular velocity to the Jacobian matrix
disp("Jacobian of the position vector p --> (Jp):");
disp(Jp);

% We need this to substitute the symbolic variables with the functions of time
% and then compute the time derivative of the Jacobian matrix --> J(q)_dot
Jp = subs(Jp, [q1_sym, q2_sym, q3_sym], [q1(t), q2(t), q3(t)]);
Jp_dot = simplify(diff(Jp, t)); % Compute the Jacobian matrix w.r.t time ----> J(q)_dot

n = simplify(Jp_dot * q_dot);                     % Compute the term n

% Desired acceleration of the end-effector
p_dot_dot = [px; py; pz]; 

% Compute the joint acceleration vector q_dot_dot
% We should isolate q_dot_dot from the equation P_dot_dot = J(q) * q_dot_dot + (J(q)_dot * q_dot)
% Hence q_dot_dot = inv(J(q)) * (P_dot_dot - (J(q)_dot * q_dot))
q_dot_dot = simplify(inv(Jp) * (p_dot_dot - n)); % Compute the joint acceleration vector q_dot_dot
disp("Joint acceleration vector q_dot_dot:");
disp(q_dot_dot);

% -- EVALUATION with numerical values
q_init = [pi/4; pi/3; -pi/2]; % Initial joint angles
q_dot_init = [-0.8; 1; 0.2];  % Initial joint velocities
p_dot_dot = [1; 1; 0];        % Desired acceleration of the end-effector

% % Substitute joint angles and their time derivatives
Jp_eval = eval(subs(Jp, q_sym, q_init));

% J(q)_dot
Jp_dot_eval = subs(Jp_dot, {diff(q1(t),t), diff(q2(t),t), diff(q3(t),t)}, {q_dot(1), q_dot(2), q_dot(3)});
Jp_dot_eval = eval(subs(Jp_dot_eval, {q1, q2, q3}, {q_init(1), q_init(2), q_init(3)}));
Jp_dot_eval = eval(subs(Jp_dot_eval, {q1_dot, q2_dot, q3_dot}, {q_dot_init(1), q_dot_init(2), q_dot_init(3)}));

% n
n_eval = subs(n, {diff(q1(t),t), diff(q2(t),t), diff(q3(t),t)}, {q_dot(1), q_dot(2), q_dot(3)});
n_eval = eval(subs(n_eval, {q1, q2, q3}, {q_init(1), q_init(2), q_init(3)}));
n_eval = eval(subs(n_eval, {q1_dot, q2_dot, q3_dot}, {q_dot_init(1), q_dot_init(2), q_dot_init(3)}));

% q_dot_dot
q_dot_dot_eval = inv(Jp_eval) * (p_dot_dot - n_eval); % Compute the joint acceleration vector q_dot_dot
disp("Joint acceleration vector q_dot_dot (evaluated):");
disp(q_dot_dot_eval);