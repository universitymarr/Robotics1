%% Exercise trajectory of 4-3-4

clear all; close all; clc

%time istants
t0=0;
t1=1;
t2=4;
tf=6;

%values at time instants
q0=0;
q1=10;
q2=80;
qf=90;

%intervals 
uno=[t0:0.01:t1];
due=[t1:0.01:t2];
tre=[t2:0.01:tf];


%Fixed matrices A,b
A=[1  t0  t0^2   t0^3     t0^4     0   0    0      0       0   0    0      0             0;
   0  1   2*t0   3*t0^2   4*t0^3   0   0    0      0       0   0    0      0             0;
   0  0   2      6*t0     12*t0^2  0   0    0      0       0   0    0      0             0;
   1  t1  t1^2   t1^3     t1^4     0   0    0      0       0   0    0      0             0;
   0  1   2*t1   3*t1^2   4*t1^3   0  -1  -2*t1  -3*t1^2   0   0    0      0             0;
   0  0   2      6*t1     12*t1^2  0   0   -2     -6*t1    0   0    0      0             0;
   0  0   0       0         0      1   t1  t1^2   t1^3     0   0    0      0             0;
   0  0   0       0         0      1   t2  t2^2   t2^3     0   0    0      0             0;
   0  0   0       0         0      0   1   2*t2   3*t2^2   0  -1  -2*t2  -3*t2^2   -4*t2^3;
   0  0   0       0         0      0   0    2     6*t2     0   0   -2    -6*t2    -12*t2^2;
   0  0   0       0         0      0   0    0      0       1   t2  t2^2   t2^3        t2^4;
   0  0   0       0         0      0   0    0      0       1   tf  tf^2   tf^3        tf^4;
   0  0   0       0         0      0   0    0      0       0   1   2*tf   3*tf^2    4*tf^3;
   0  0   0       0         0      0   0    0      0       0   0    2     6*tf     12*tf^2];



b=transpose([q0  0  0  q1  0  0  q1  q2  0  0  q2  qf  0  0]);



%x=[al0  al1  al2  al3  al4  at0  at1  at2  at3  as0  as1  as2  as3  as4]
x=inv(A)*b

%parameters of trajectories
al0=x(1);    at0=x(6);    as0=x(10);
al1=x(2);    at1=x(7);    as1=x(11);
al2=x(3);    at2=x(8);    as2=x(12);
al3=x(4);    at3=x(9);    as3=x(13);
al4=x(5);                 as4=x(14);


%position trajectories
qL=al0 + al1*uno + al2*(uno.^2) + al3*(uno.^3) + al4*(uno.^4);
qT=at0 + at1*due + at2*(due.^2) + at3*(due.^3);
qS=as0 + as1*tre + as2*(tre.^2) + as3*(tre.^3) + as4*(tre.^4);

%velocity trajectories
dotqL=al1 + 2*al2*uno + 3*al3*(uno.^2) + 4*al4*(uno.^3);
dotqT=at1 + 2*at2*due + 3*at3*(due.^2);
dotqS=as1 + 2*as2*tre + 3*as3*(tre.^2) + 4*as4*(tre.^3);

%acceleration trajectories
ddotqL=2*al2 + 6*al3*uno + 12*al4*(uno.^2);
ddotqT=2*at2 + 6*at3*due;
ddotqS=2*as2 + 6*as3*tre + 12*as4*(tre.^2);

title("Position trajectories");
hold on
plot(uno,qL);
plot(due,qT);
plot(tre,qS);

pause;
hold off

figure;
title("Velocity trajectories");
hold on
plot(uno,dotqL);
plot(due,dotqT);
plot(tre,dotqS);

pause;
hold off

figure;
title("Acceletarion trajectories");
hold on
plot(uno,ddotqL);
plot(due,ddotqT);
plot(tre,ddotqS);










