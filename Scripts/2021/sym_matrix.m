clear all
%% Define symbolic variables
syms alpha a d theta
syms q1 q2 q3 q4 q5 q6
syms  d1 a2 a3 d4 L

%% number of joints
N=3;

%% Insert DH table of parameters of SCARA
DHTABLE = [0 0 0 q1;
           pi/2 0 q2 0;
           0 0 L q3;]
    
%% Build the general Denavit-Hartenberg trasformation matrix
TDH = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
       sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];
      
%% Build transformation matrices for each link

% empty cell array, we will put homogenous matrices
A = cell(1,N);  

% For every row in 'DHTABLE' we substitute the right value inside
% the general DH matrix
for i = 1:N
    alpha = DHTABLE(i,1);
    a = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
end

for i = 1:N
    fprintf('A{%d} \n',i);
    % in this case we show all homogenous tranformation matrix
    A{i};
    pause;
end

% we compute the final result matrix
% homogenous matrix of last frame respect to the first frame
tot=A{1};
for i = 2:N
    ris=A{i};
    tot=tot*ris;
end

%% Compute the point of last frame respect to the first frame
p=simplify(tot(1:3,4))
pause;

%% Compute matrices R from homogenous A
R = cell(1,N);  

for i = 1:N
    R{i}=A{i}(1:3,1:3);
    fprintf('R{%d} \n',i);
    R{i};
    pause;
end

%% Analytic Jacobian
AJ = jacobian(p,[q1 q2 q3])