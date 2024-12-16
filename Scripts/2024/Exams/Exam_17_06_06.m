% Global variables
syms q1 q2 q3 q4 L B
n_joints = 4;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [ -pi/2  0   q1     pi/2   ;
             -pi/2  0    0     q2     ;
              pi/2  q3   0     0      ;
              0     0    L     q4     ;
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

%% evaluate Robot in a given configuration and/or position distance
P = A{1} * (A{2} * (A{3} * (A{4} * [0, 0, 0, 1].')));
P = simplify(P);
disp('P:')
disp(P)

% We could've also used the following form:
% P = T * [0, 0, 0, 1].'
%
% That's because T is actually 0^A_4.
%
% -- Remember:
%   A{1} = 0^A_1
%   A{2} = 1^A_2
%   A{3} = 2^A_3
%   A{4} = 3^A_4