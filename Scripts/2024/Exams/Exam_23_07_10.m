%% -------------------- Exercise 1 --------------------

% Global variables
syms b1 b2 b3
syms L3
n_joints = 3;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [ 0    0  0  b1;
             0   b2  0   0;
             0   L3  0  b3;
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

%% -------------------- Exercise 3 --------------------
% Analytical Solution of Inverse Kinematics

% Given values
L2 = 2.99;
x = 1.8;
y = 1.2;
alpha = 1.5708;
pos_neg = "pos";

% call the function
joint_values = inverse_kinematics_RPR(L2, x, y, alpha, pos_neg);

%% -------------------- Exercise 6 --------------------
variables = [b1, b2, b3];
J = DH_to_JL(DH_table, variables);

% Since it's not a square matrix, for computing the determinant, we use J.T * J
detJ = simplify(det(J));

% For finding the singularities, we have to solve the equation det(J.) = 0
% We can use the solve function to find the singularities w.r.t q1, q2, q3
[b1_sol, b2_sol, b3_sol] = solve(detJ == 0, variables);
b1_sol = simplify(b1_sol);
b2_sol = simplify(b2_sol);
b3_sol = simplify(b3_sol);

%% -------------------- Exercise 7 --------------------
J_singular = subs(J, variables, [b1_sol, b2_sol, b3_sol]);

% Compute the nullspace of the Jacobian
nullspace_J = null(J_singular);
nullspace_J_transposed = null(J_singular.');

% Display the nullspace of the Jacobian
disp('Nullspace of the Jacobian N(J(Qs)):');
disp(simplify(nullspace_J));
disp('Nullspace of the Transposed Jacobian N(J^(Qs)):');
disp(simplify(nullspace_J_transposed));

% Compute the range of the jacobian
range = simplify(orth(sym(J_singular)));
range_transposed = simplify(orth(sym(J_singular.')));

% Display the range
disp('Basis of R(J(Qs)):');
% Display the basis of the range
[~,m]=size(range);
for i=1:m
    [numerator, ~] = numden(range(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end

disp('Basis of R(J^T(Qs)):');
% Display the basis of the range
[~,m]=size(range_transposed);
for i=1:m
    [numerator, ~] = numden(range_transposed(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end
