%% -------------------- Exercise 1 --------------------

% Global variables
syms q1 q2 q3 L M a1
n_joints = 3;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [  pi/2,        a1       ,  0, q1;
             -pi/2,         0       , q2,  0;
                 0,  sqrt(L^2 + M^2),  0, q3;
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
T_0 = subs(T, {q1, q2, q3}, {0, 0, 0});

disp('Position vector p for q1 = q2 = q3 = 0:');
disp(T_0(1:3, 4));

disp('Rotation matrix R for q1 = q2 = q3 = 0:');
disp(T_0(1:3, 1:3));

disp("-------------------------------------------------------------------")