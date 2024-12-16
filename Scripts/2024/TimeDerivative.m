% % Original way of computing the Analytic Jacobian
% syms l1 l2 q1(t) q2(t)
% 
% px = l1 * cos(q1) + l2 * cos(q1+q2);
% py = l1 * sin(q1) + l2 * sin(q1+q2);
% phi = q1 + q2;
% 
% px_dot = simplify(diff(px, t));  % First derivative of px w.r.t. time
% py_dot = simplify(diff(py, t));  % First derivative of px w.r.t. time
% phi_dot = simplify(diff(phi, t));  % First derivative of px w.r.t. time

% We do not need to differentiate w.r.t. time.
% We can directly have the Jacobian we need by calling jacobian(x,y)
% with the proper arguments, by differentiating w.r.t. q1, q2
syms l1 l2 q1 q2

p = [l1 * cos(q1) + l2 * cos(q1+q2);  % px
     l1 * sin(q1) + l2 * sin(q1+q2);  % py
     q1 + q2];                        % phi

% Compute the Jacobian matrix by differentiating w.r.t. q1 and q2
J_r = jacobian(p, [q1, q2]);

disp("-------------------------------------------------------------------")
disp('J_r(q):')
disp(J_r)
disp("-------------------------------------------------------------------")

% Define the time derivatives of q1 and q2 as a column vector
syms q1(t) q2(t)

%syms x(t)
%q1 = sin(x);
%q2 = cos(x);

q1_dot = simplify(diff(q1, t));
q2_dot = simplify(diff(q2, t));
q_dot = [q1_dot; q2_dot];

% Compute r_dot by multiplying J_r by q_dot
r_dot = J_r * q_dot;

disp('r_dot:')
disp(r_dot)