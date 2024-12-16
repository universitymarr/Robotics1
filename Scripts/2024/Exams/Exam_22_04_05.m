%% -------------------- Exercise 1 --------------------

% Global variables
syms q1 q2 q3 q4 A B C
n_joints = 4;

% alpha, a_i, d_i, theta_i
syms alpha a d theta
DH_table = [ -pi/2,  0,  A  q1;
             -pi/2,  0,  0  q2;
              pi/2,  0,  B  q3;
                 0,  C,  0, q4;
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

%% 2nd bullet point

% Extract the transformation matrix 4TE from the previous exercise (from frame RF4 to frame RFE)
% -- Remember:
%   A{1} = 0^A_1
%   A{2} = 1^A_2
%   A{3} = 2^A_3
%   A{4} = 3^A_4
%   ...
%   A{i} = i-1^A_i

RF4_TO_RFE = A{4};
disp('4TE:')
disp(RF4_TO_RFE)

disp("-------------------------------------------------------------------")

%% 3rd bullet point
RF0_TO_RFE = T;
disp('0TE:')

variables = symvar(RF0_TO_RFE);
values = [1, 1, 1, pi/2, pi/2, 0, 0]; % Values for: A, B, C, q1, q2, q3, q4

RF0_TO_RFE_subbed = subs(RF0_TO_RFE, variables, values);
disp('RF0_TO_RFE with substitutions for present variables:')
disp(RF0_TO_RFE_subbed)

disp("-------------------------------------------------------------------")

%% 4th bullet point
JA = DH_to_JA(DH_table, []);
jacobian_analysis(JA, variables)

% 5th bullet point
% Find the symbolic expression (as a function of the configuration q) of a non-trivial joint velocity ˙ q0= 0 such that ωE = JA(q)˙ q0 = 0 for all possible q.
nullspace_J = simplify(null(JA));
disp('Nullspace of the Jacobian (namely, q0):')
disp(nullspace_J)

disp('JA(q) * q0 (This should give 0):')
disp(simplify(JA * nullspace_J))

% ---------------------- Exercise 2 ----------------------

% First segment (0 to T/8)
syms t J T
q_dot_dot_dot = J;

q_dot_dot = simplify(int(q_dot_dot_dot, t));
q_dot = simplify(int(q_dot_dot, t));
q = simplify(int(q_dot, t));

disp('FIRST SEGMENT -- [q_dot_dot, q_dot, q] when q(t):');
disp([q_dot_dot, q_dot, q]);

T_8 = T/8;
q_dot_dot_T8 = simplify(subs(q_dot_dot, t, T_8));
q_dot_T8 = simplify(subs(q_dot, t, T_8));
q_T8 = simplify(subs(q, t, T_8));

disp('[q_dot_dot, q_dot, q] when q(T/8):');
disp([q_dot_dot_T8, q_dot_T8, q_T8]);

% Second segment (T/8 to T/4)
syms C1 C2 C3
q_dot_dot_dot_2 = -J;

q_dot_dot_2 = int(q_dot_dot_dot_2, t) + C1;
q_dot_2 = int(q_dot_dot_2, t) + C2;
q_2 = int(q_dot_2, t) + C3;

% Solve for C1, C2, and C3 using conditions at T/8
eq1 = subs(q_dot_dot_2, t, T_8) == q_dot_dot_T8;
eq2 = subs(q_dot_2, t, T_8) == q_dot_T8;
eq3 = subs(q_2, t, T_8) == q_T8;
sol = solve([eq1, eq2, eq3], [C1, C2, C3]);

% Substitute the solved constants
q_dot_dot_2 = subs(q_dot_dot_2, [C1, C2, C3], [sol.C1, sol.C2, sol.C3]);
q_dot_2 = subs(q_dot_2, [C1, C2, C3], [sol.C1, sol.C2, sol.C3]);
q_2 = subs(q_2, [C1, C2, C3], [sol.C1, sol.C2, sol.C3]);

disp('SECOND SEGMENT -- [q_dot_dot, q_dot, q] when q(t):');
disp([simplify(q_dot_dot_2), simplify(q_dot_2), simplify(q_2)]);

% Values at T/4
T_4 = T/4;
q_dot_dot_T4 = simplify(subs(q_dot_dot_2, t, T_4));
q_dot_T4 = simplify(subs(q_dot_2, t, T_4));
q_T4 = simplify(subs(q_2, t, T_4));

disp('Values at T/4 [q_dot_dot, q_dot, q]:');
disp([q_dot_dot_T4, q_dot_T4, q_T4]);

% Third segment (T/4 to T/2)
syms D1 D2 D3
q_dot_dot_dot_3 = 0;

q_dot_dot_3 = D1;  % Constant acceleration
q_dot_3 = D1*t + D2;
q_3 = (1/2)*D1*t^2 + D2*t + D3;

% Solve for D1, D2, and D3 using conditions at T/4
eq1 = subs(q_dot_dot_3, t, T_4) == q_dot_dot_T4;
eq2 = subs(q_dot_3, t, T_4) == q_dot_T4;
eq3 = subs(q_3, t, T_4) == q_T4;
sol = solve([eq1, eq2, eq3], [D1, D2, D3]);

% Substitute the solved constants
q_dot_dot_3 = subs(q_dot_dot_3, [D1, D2, D3], [sol.D1, sol.D2, sol.D3]);
q_dot_3 = subs(q_dot_3, [D1, D2, D3], [sol.D1, sol.D2, sol.D3]);
q_3 = subs(q_3, [D1, D2, D3], [sol.D1, sol.D2, sol.D3]);

disp('THIRD SEGMENT -- [q_dot_dot, q_dot, q] when q(t):');
disp([simplify(q_dot_dot_3), simplify(q_dot_3), simplify(q_3)]);

% Values at T/2
T_2 = T/2;
q_dot_dot_T2 = simplify(subs(q_dot_dot_3, t, T_2));
q_dot_T2 = simplify(subs(q_dot_3, t, T_2));
q_T2 = simplify(subs(q_3, t, T_2));

disp('Values at T/2 [q_dot_dot, q_dot, q]:');
disp([q_dot_dot_T2, q_dot_T2, q_T2]);