%% compute Geometric Jacobian after run "sym_matrix" because this geometric jacobian
%% is computed with DH
disp('third case: Geometric Jacobian JL and JA')

% compute homogenous matrix of second frame respect to the initial frame
A12=A{1}*A{2};

% compute homogenous matrix of third frame respect to the initial frame
A13=A12*A{3};

% compute homogenous matrix of third frame respect to the initial frame
% you remove the comment if you have N=4 or 5 depends of number of joints
% A14=A13*A{4};
% A15=A14*A{5}

%% compute z for JL and JA
z0= [0 0 1];
z1=A{1}(1:3,3);
z2=A12(1:3,3);
% z3=A13(1:3,3);
% z4=A14(1:3,3);

%% points of JL
p0=[0; 0; 0;];
p1=A{1}(1:3,4);
p2=A12(1:3,4);
p3=A13(1:3,4);
% p4=A14(1:3,4);
% p5=A15(1:3,4);

%% JL
J(1:3,1)=cross(z0,p3-p0);
J(1:3,2)=cross(z1,p3-p1);
J(1:3,3)=cross(z2,p3-p2);
% J(1:3,4)=cross(z3,p4-p3);
% J(1:3,5)=z4;

%% JA
J(4:6,1)=z0;
J(4:6,2)=z1;
J(4:6,3)=z2;
% J(4:6,4)=z3;
% J(4:6,5)=0;

%% Total J = [JL; JA]
J= simplify(J)
pause;

%% print JL
JL= simplify(J(1:3,1:3))
pause;

%% print JA
JA= simplify(J(4:6,1:3))