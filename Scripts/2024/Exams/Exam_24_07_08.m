% -=================================-
% Exam: 08/07/2024                  |
% Student: Gianmarco Scarano        |
% ID: 2047315                       |  
% -=================================-

% ------------------- Exercise 2 -------------------
syms gamma(t) beta(t) alpha(t) rx ry rz % Define all the symbolic variables
gamma_sym = sym('gamma');
beta_sym = sym('beta');
alpha_sym = sym('alpha');

R = [sqrt(2)/2, 0, sqrt(2)/2;
     sqrt(2)/2, 0, -sqrt(2)/2;
     0, 1, 0];

w_eval = [1; 1; 2];

[alpha_v, beta_v, gamma_v] = euler_rotation_inverse(char("XZY"), R, "pos");

Rx = [1,               0,                0;
      0,  cos(alpha_sym),  -sin(alpha_sym);
      0,  sin(alpha_sym),   cos(alpha_sym);
     ];

Ry = [ cos(beta_sym),   0,   sin(beta_sym);
                   0,   1,               0;
      -sin(beta_sym),   0,   cos(beta_sym);
     ];

Rz = [cos(gamma_sym),  -sin(gamma_sym),   0;
      sin(gamma_sym),   cos(gamma_sym),   0;
                   0,                0,   1;
     ];

% Rotation around Y(a)Z(b)X(g), so in RPY type is X(g)Z(b)Y(a)
Ry_exercise = subs(Ry, beta_sym, alpha_sym);
Rz_exercise = subs(Rz, gamma_sym, beta_sym);
Rx_exercise = subs(Rx, alpha_sym, gamma_sym);
R_exercise = simplify(Rx_exercise * Rz_exercise * Ry_exercise);

% Now, since we have to compute the S(w) = R_dot * R.T,
% we have to compute the derivative of R with respect to time
R_var = subs(R_exercise, [alpha_sym, beta_sym, gamma_sym], [alpha(t), beta(t), gamma(t)]);
R_dot = diff(R_var, t);
R_dot = simplify(R_dot);
disp('Time derivative of the rotation matrix R:');
disp(R_dot);
disp("----------------------------------");

% Compute the skew-symmetric matrix S(w) = R_dot * R.T
S_w = simplify(R_dot * simplify(transpose(R_var)));
disp('Skew-symmetric matrix S(w) = R_dot * R.T:');
disp(S_w);
disp("----------------------------------");

% We can now extract the w vector from the matrix, taking these elements:
% skew_matrix = [  0,  -rz,   ry;
%                 rz,    0,  -rx;
%                -ry,   rx,    0;
% ];
% wx = S_w(3, 2)
% wy = S_w(1, 3)
% wz = S_w(2, 1)
disp('Angular velocity vector w = [wx; wy; wz]:');
w = [simplify(S_w(3, 2)); simplify(S_w(1, 3)); simplify(S_w(2, 1));];
disp(w);
disp("----------------------------------");

% Substitute the time derivatives
syms alpha_dot beta_dot gamma_dot
w_subs = subs(w, {diff(alpha(t), t), diff(beta(t), t), diff(gamma(t), t)}, {alpha_dot, beta_dot, gamma_dot});

% Define the vector of time derivatives
phi_dot = [alpha_dot; beta_dot; gamma_dot];

% Solve for the coefficients of each time derivative
T_phi = equationsToMatrix(w_subs, phi_dot);

disp('T(phi) = ');
disp(simplify(T_phi));
disp('phi_dot =')
disp(simplify(phi_dot));
disp("----------------------------------");

% omega = T(phi) * phi_dot
% So we want to know T(phi) for then finding phi_dot.
T_phi_eval = double(subs(T_phi, {alpha(t), beta(t), gamma(t)}, {alpha_v, beta_v, gamma_v}));
phi_dot_eval = inv(T_phi_eval) * w_eval;
disp("Phi dot:"); disp(phi_dot_eval);

% ------------------- Exercise 3 -------------------
clear all;
qa = [pi/4, -pi/3];

% Compute the direct kinematics for Robot A
px = cos(qa(1)) + cos(qa(1) + qa(2));
py = sin(qa(1)) + sin(qa(1) + qa(2));
disp("Direct kinematics for Robot A:");
disp([px, py]);

% Define symbolic variables
syms x y

% Define the equations
eq1 = (x - px)^2 + (y - py)^2 == 4;
eq2 = y == -x + 4;

% Solve the system of equations
solution = solve([eq1, eq2], [x, y]);

% Extract the solutions
x_solutions = double(solution.x);
y_solutions = double(solution.y);

% Display the solutions
for i = 1:length(x_solutions)
    fprintf('Solution %d:\n', i);
    fprintf('x = %.4f\n', x_solutions(i));
    fprintf('y = %.4f\n\n', y_solutions(i));
end

xNew_b = px - x_solutions(1);
yNew_b = py - y_solutions(1);
disp("New coordinates for Robot B:");
disp([xNew_b, yNew_b]);

[newAngles] = inverse_kinematics_2R_planar(1, 1, xNew_b, yNew_b, "pos");
disp("New angles for Robot B:");
disp(newAngles);

xNew_b_otherSolution = px - x_solutions(2);
yNew_b_otherSolution = py - y_solutions(2);

c2 = (xNew_b_otherSolution^2 + yNew_b_otherSolution^2 - (1^2 + 1^2)) / (2 * 1 * 1);
s2 = 0; % Approximated from sqrt(1 - c2^2) which is not doable

q2 = atan2(s2, c2);
q1 = atan2(yNew_b_otherSolution, xNew_b_otherSolution) - atan2(1 * s2, 1 + 1 * c2);
newAngles_otherSolution = [q1, q2];
disp("New angles for Robot B (other solution):");
disp(newAngles_otherSolution);

% ------------------- Exercise 4 -------------------
clear all;
pi = [0.6; -0.3];
pf = [-0.3; 0.6];
q1_dot = 2;
q2_dot = 1;
A = 0.5;

% Compute the inverse kinematics of RP for joint q1/q2 at pi/pf
q2_pi = sqrt(pi(1)^2 + pi(2)^2);
q1_pi = atan2(pi(2), pi(1));

q2_pf = sqrt(pf(1)^2 + pf(2)^2);
q1_pf = atan2(pf(2), pf(1));

disp("Initial joint angles:"); disp([q1_pi, q2_pi]);
disp("Final joint angles:"); disp([q1_pf, q2_pf]);

delta_q1 = q1_pf - q1_pi;
delta_q2 = q2_pf - q2_pi;
disp("Delta q1:"); disp(delta_q1);
disp("Delta q2:"); disp(delta_q2);

Tmin_q1 = (delta_q1 * A) + (q2_dot^2) / (A * q2_dot);
Tmin_q2 = (delta_q2 * A) + (q1_dot^2) / (A * q1_dot);
disp("Tmin q1:"); disp(Tmin_q1);
disp("Tmin q2:"); disp(Tmin_q2);