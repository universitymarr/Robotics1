syms q1 q2  dq1 dq2

%% Inverse kinematics of 2R Planar Robot

% length l
l1=0.5
l2=0.25

% x of p and y of p
px=0.35
py=0.3


% p in planar 2R
p=[l1*cos(q1)+l2*cos(q1+q2);
    l1*sin(q1)+l2*sin(q1+q2)]

% jacobian of o
J=jacobian(p,[q1 q2])
pause;

% second joint computations
c2=(px^2+py^2-l1^2-l2^2)/(2*l1*l2);
s2pos=sqrt(1-c2^2);  
%other solution: -sqrt(1-c2^2)
s2neg=-sqrt(1-c2^2);  

% first joint computations
detM=l1^2+l2^2+2*l1*l2*c2;

% positive solution of q1
s1pos=(py*(l1+l2*c2)-px*l2*s2pos)/detM;
c1pos=(px*(l1+l2*c2)+py*l2*s2pos)/detM;

% negative solution of q1
s1neg=(py*(l1+l2*c2)-px*l2*s2neg)/detM;
c1neg=(px*(l1+l2*c2)+py*l2*s2neg)/detM;

% output positive
q01p=atan2(s1pos,c1pos)
q02p=atan2(s2pos,c2)
q0p=[q01p; q02p]
pause;

% output negative
q01n=atan2(s1neg,c1neg)
q02n=atan2(s2neg,c2)
q0n=[q01n; q02n]
pause;

% time derivative of the Jacobian
Jder=[-l1*cos(q1)*dq1-l2*cos(q1+q2)*(dq1+dq2) -l2*cos(q1+q2)*(dq1+dq2);
    -l1*sin(q1)*dq1-l2*sin(q1+q2)*(dq1+dq2) -l2*sin(q1+q2)*(dq1+dq2)]
pause;



