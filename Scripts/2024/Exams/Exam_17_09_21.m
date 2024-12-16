% -------------------------------------------------------------------------
syms a b y

R_z = [cos(a), -sin(a), 0;
        sin(a), cos(a), 0;
        0, 0, 1];
R_x = [1, 0, 0;
        0, cos(b), -sin(b);
        0, sin(b), cos(b)];
R_y = [cos(y), 0, sin(y);
        0, 1, 0;
        -sin(y), 0, cos(y)];

R = simplify(R_z * R_x * R_y)

% We need to substitute the following values:
% a = 30 degrees
% b = -30 degrees
% y = 60 degrees
% We need to express these angles in radians
Evaluated_R = subs(R, [a, b, y], [30 * pi / 180, -30 * pi / 180, 60 * pi / 180]);
Evaluated_R = eval(Evaluated_R) % Substitute the values in the matrix

% -------------------------------------------------------------------------
syms alpha_dot beta_dot gamma_dot
% Compute the transformed vectors
v1 = [0, 0, 1].';
v2 = R_z * [1; 0; 0]; 
v3 = R_z * R_x * [0; 1; 0];

% Concatenate them to form the omega matrix
omega = [v1, v2, v3];
omega = omega * [alpha_dot; beta_dot; gamma_dot];
omega = simplify(omega);

% Display the result
disp('omega =');
disp(omega);

% Rotate the omega matrix to the final frame
R_omega = (R_z * R_x * R_y).' * omega;
R_omega = simplify(R_omega);
disp('R_omega =');
disp(R_omega);
% -------------------------------------------------------------------------