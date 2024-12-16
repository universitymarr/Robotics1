%% EXERCISE 1

% Global variables
syms q1 q2 q3 q4
n_joints = 4;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [  pi/2  0  q1  -pi/2 ;
             -pi/2  0  q2   pi/2 ;
              pi/2  0  q3  -pi/2 ;
             -pi/2  0  q4   pi/2 ;
            ];

DH = DHmatrix(alpha, a, d, theta);

A = cell(1,n_joints);

% Compute each transformation matrix
for i = 1: n_joints
    A{i} = subs(DH, {alpha, a, d, theta}, DH_table(i, :));
end

T = eye(4);

disp("-------------------------------------------------------------------")
% -- Remember:
%   A{1} = 0^A_1
%   A{2} = 1^A_2
%   A{3} = 2^A_3
%   A{4} = 3^A_4
%   ...
%   A{i} = i-1^A_i
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

%% EXERCISE 2

W_T_0 = [0,0,1,0;
         1,0,0,0;
         0,1,0,0;
         0,0,0,1];

disp('W^T_0:')
disp(W_T_0)

F4_T_E = A{4};
disp('4^T_E:')
disp(F4_T_E)

%% EXERCISE 3

W_T_E = W_T_0 * A{1} * A{2} * A{3} * A{4};
disp('W^T_E:')
disp(W_T_E)

disp("-------------------------------------------------------------------")

%% EXERCISE 4

% Task Jacobian
px = W_T_E(1,4);
py = W_T_E(2,4);
variables = [q1; q2; q3; q4];

J = jacobian([px; py], variables);

disp('Jacobian:');
disp(J);

% Determinant
disp('Determinant:');
detJ = simplify(det((J * J.')));
disp(detJ);

% Find singularities
disp('Singularities:');
disp(detJ == 0);

% For finding the singularities, we have to solve the equation det(J) = 0
% We can use the solve function to find the singularities w.r.t q1, q2, q3, q4
[q1_sol, q2_sol, q3_sol, q4_sol] = solve(detJ == 0, variables);
q1_sol = simplify(q1_sol);
q2_sol = simplify(q2_sol);
q3_sol = simplify(q3_sol);
q4_sol = simplify(q4_sol);

%% EXERCISE 5

% Basis for N(J)
% Reduce the null space basis vectors to row echelon form.
% We also could've directly used null(J) to get the basis, but rref retrieves the basis in a more readable form.
% -- TO BE CHECKED
nullspaceJ = null(J);
baseN_J = rref(nullspaceJ);
disp('Basis for N(J):');
disp(nullspaceJ);
disp('Row echelon form of the null space basis:');
disp(baseN_J);

% Basis for R(J') where R is the Range
disp('Basis for R(J''):');
disp(orth(sym(J.')));

%% EXERCISE 6
r_dot = [3, -2].';
q_dot = pinv(J) * r_dot;
disp('q_dot:');
disp(q_dot);

%% EXERCISE 7
F = [2, 1].';
torque = J.' * F;
disp('Torque:');
disp(torque);

%% EXERCISE 8 on paper