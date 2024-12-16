%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% you first have to run "sym_matrix" because this geometric jacobian is computed with DH

%then, accordingly to the number of joints, you have to change in this file:

%the number of homogenous matrices wrt the initial frame, following
% A1i = Ai-1*A{i} for i>2

% the number of z computed, following
% zi = A{i}(1:3, 3); for i>1

% the number of points of JL, following
% pi=A1i(1:3,4); for i>1

% the JL and JA for revolute/prismatic joint
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


disp('third case: Geometric Jacobian JL and JA')

% compute homogenous matrices with respect to the initial frame
A12=A{1}*A{2};
A13=A12*A{3};
%A14=A13*A{4};
%A15=A14*A{5};
%A16=A15*A{6};

%% compute z for JL and JA
z0= [0 0 1];
z1=A{1}(1:3,3);
z2=A12(1:3,3);
% z3=A13(1:3,3);
% z4=A14(1:3,3);
% z5=A15(1:3,3);

%% points of JL
p0=[0; 0; 0;];
p1=A{1}(1:3,4);
p2=A12(1:3,4);
p3=A13(1:3,4);
% p4=A14(1:3,4);
% p5=A15(1:3,4);
% p6=A16(1:3,4);

%% JL
%% prismatic joint: zi-1
%% revolute joint: cross(zi-1 x pe-pi-1)
J(1:3,1)=cross(z0,p3-p0);
J(1:3,2)=cross(z1,p3-p1);
J(1:3,3)=cross(z2,p3-p2);
% J(1:3,4)=cross(z3,p4-p3);
% J(1:3,5)=z4;
% J(1:3,6)=sym(cross(z5,p6-p5));

%% JA
%%prismatic joint: 0
%% revolute joint zi-1
J(4:6,1)=z0;
J(4:6,2)=z1;
J(4:6,3)=z2;
% J(4:6,4)=z3;
% J(4:6,5)=0;
% J(4:6,6)=z5;

%% Total J = [JL; JA]
J= simplify(J)
pause;

%% print JL
JL= simplify(J(1:3,:))
pause;

%% print JA
JA= simplify(J(4:6,:))
