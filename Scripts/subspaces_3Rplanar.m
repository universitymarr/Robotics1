% Subspaces (images and kernels) of the Jacobian of a robot and their dimensions
% Example: planar 3R robot with equal (unitary) links
% by A. De Luca, first 25 Nov 2011 
% revised 3 Dec 2018 (run twice: for regular and singular cases)
% available in the course material of Robotics 1

clear all
clc

syms q1 q2 q3 q4 l1 l2 l3
%% Subspace analysis (range and null spaces): R(J), N(J), R(J^T), N(J^T)
disp('Subspace analysis (range and null spaces): R(J), N(J), R(J^T), N(J^T)')
disp('of the robot Jacobian (a mxn matrix) in a given configuration q')
pause
disp(' ')
disp('robot: planar 3R arm with unitary links --> joints (n=3)')
disp('task: end-effector position in the plane --> dimension of cartesian space (m=2)')
pause
disp(' ')
disp('direct (task) kinematics')

%% values of links 
l1=1;
l2=0.5;
l3=0.25;

%% compute p 
p=[l1*cos(q1)+l2*cos(q1+q2)+l2*cos(q1+q2+q3);
    l1*sin(q1)+l2*sin(q1+q2)+l3*sin(q1+q2+q3);
    ;];

%% compute values of angle
phi= [q1+q2+q3];

%% f(q) = r
f= [p; phi;];

% vector we use to compute jacobian matrix
x=[q1 q2 q3];

disp('Jacobian (a 6x3 matrix)')
%% jacobian matrix
Jac=simplify(jacobian(f,x))
pause;

%% to subs value in jacobian matrix 
J=subs(Jac,{q1,q2,q3},{0,0,0})
pause;

% compute rank of J to see if it maximum
% if it maximum there no null Space otherwise we have it 
rankJ=rank(J)
pause

% but if you're running this code it means it exists :)

%% Compute Null Space and Range Space of Jacobian J

%% Null Space of J
NullSpaceJ=null(J)
dimNullSpaceJ=size(NullSpaceJ,2);
disp('normalizing...')
if dimNullSpaceJ>1,
   NullSpaceJ(:,1)=NullSpaceJ(:,1)/norm(NullSpaceJ(:,1));
   NullSpaceJ(:,2)=NullSpaceJ(:,2)/norm(NullSpaceJ(:,2));
   NullSpaceJ
else
   NullSpaceJ=NullSpaceJ/norm(NullSpaceJ);
end
NullSpaceJ=simplify(NullSpaceJ)
pause;

% dim Null space
dimNullSpaceJ=size(NullSpaceJ,2)
pause;

% verify if J*NullSpace give us vector 0
disp('check null-space joint velocity: J*NullSpaceJ ?')
simplify(J*NullSpaceJ)
pause;


%% Range Space of J
RangeJ=simplify(orth(J))
pause;

% dim Range space 
dimRangeSpaceJ=size(RangeJ,2)
pause;

%% Compute Null Space and Range Space with transpose of J
disp('b) working with the Jacobian transpose')
JT=J'
pause

% rank of range
rankJT=rank(JT)
pause

%% Null Space of transpose of J
NullSpaceJT=null(JT);
dimNullSpaceJT=size(NullSpaceJT,2);
if dimNullSpaceJT>0,
   NullSpaceJT=simplify(NullSpaceJT/norm(NullSpaceJT));
else
   NullSpaceJT
end
pause;

% dim null space
dimNullSpaceJT=size(NullSpaceJT,2)
pause;

%% Range Space of transpose of J
RangeJT=orth(JT);
RangeJT=simplify(RangeJT)
pause;

% dim range space
dimRangeSpaceJT=size(RangeJT,2)
pause;

%% check dimension of joint space and task space
disp('c) final check on dimensions of subspaces')

disp(' ')
disp('dim Range(J) + dim Null(J^T) ?')
pause 
dimRangeSpaceJ+dimNullSpaceJT
disp('..equal to the task-space dimension')
pause

disp(' ')
disp('dim Range(J^T) + dim Null(J) ?')
pause
dimRangeSpaceJT+dimNullSpaceJ
disp('..equal to the joint-space dimension')

% end