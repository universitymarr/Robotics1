% QUESTION 1 - First bullet point
%
% Global variables
syms q1 q2 q3 L1 L2 L3
n_joints = 3;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [ pi/2   0   L1     q1   ;
                0  L2    0     q2   ;
                0  L3    0     q3   ;
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

% QUESTION 1 - Second bullet point
%% Inverse Kinematics problem for 3R planar manipulator
% Given the position vector p = [px, py, pz] and the orientation of the end-effector
% represented by the rotation matrix R = [nx, sx, ax; ny, sy, ay; nz, sz, az]
% Find the joint variables q1, q2, q3

% Given position vector p
p = T(1:3, 4);
px = p(1);
py = p(2);
pz = L3 * sin(q2 + q3) + L2 * sin(q2); % We remove L1 because we want to transfer it into the left-hand side of the equation

% Square and sum the position vector components
squareSum = simplify(px^2 + py^2 + (pz - L1)^2) == 0;

% Isolate cos(q3)
cos_q3_expression = isolate(squareSum, cos(q3));
disp(cos_q3_expression); % Display the expression for cos(q3)

% Retrieve sin(q3) from the expression for cos(q3)
rightHandSide_cos_q3_expression = rhs(cos_q3_expression); % Extract the right-hand side of the equation for cos(q3)
sin_q3_expression = simplify(sqrt(1 - rightHandSide_cos_q3_expression^2));
disp("sin(q3):")
disp(sin_q3_expression); % Display the expression for sin(q3)

% Now simply calculate q3 using the atan2
q3 = simplify(atan2(sin_q3_expression, rightHandSide_cos_q3_expression));
disp("q3:")
disp(q3); % Display the expression for q3

disp("-------------------------------------------------------------------")

% QUESTION 2 - First bullet point

syms V A
qa = 5;
qb = 10;
delta = qb - qa;
T_min = (delta * A + V^2) / (A * V);
T_max = delta / V;
