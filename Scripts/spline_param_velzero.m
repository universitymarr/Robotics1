%% spline plot in case we have initial and final velocity are zero

clear all; close all; clc
syms v2 v3

% time istants
t1=0;
t2=0.25;
t3=0.5;
t4=1;

% values at time instants
q1=70;
q2=-45;
q3=10;
q4=100;

% intervals 
t12=[t1:0.01:t2];
t23=[t2:0.01:t3];
t34=[t3:0.01:t4];

% tau interval
tau1=(t12-t1)/(t2-t1);
tau2=(t23-t2)/(t3-t2);
tau3=(t34-t1)/(t2-t2);

%% if v1=0 and v4=0
a1=0; a2=3*(q2-q1)-v2*(t2-t1); a3= v2*(t2-t1)-2*(q2-q1);
b1=v2*(t3-t2); b2=3*(q3-q2)-(2*v2+v3)*(t3-t2); b3= -2*(q3-q2)+(v2+v3)*(t3-t2);
c1=v3*(t4-t3); c2=3*(q4-q3)-2*v3*(t4-t3); c3=v3*(t4-t3)-2*(q4-q3);

% matrix A where Ax=b
A=[4*(t3-t1) 2*(t2-t1);
    2*(t4-t3) 4*(t4-t2);]

% matrix b
b=[(6*(q3-q2)*(t2-t1))/(t3-t2) + (6*(q2-q1)*(t3-t2))/(t2-t1);
    (6*(q3-q2)*(t4-t3))/(t3-t2) + (6*(q4-q3)*(t3-t2))/(t4-t3)]

% x contains two vector v2 and v3
x=inv(A)*b

% two vector
v22=x(1)
v33=x(2)

pause;
% subs coefficients
a1=subs(a1,{v2},{v22}); a2=subs(a2,{v2},{v22}); a3=subs(a3,{v2},{v22});
b1=subs(b1,{v2},{v22}); b2=subs(b2,{v2,v3},{v22,v33}); b3=subs(b3,{v2},{v22,v33});
c1=subs(c1,{v3},{v33}); c2=subs(c2,{v3},{v33}); c3=subs(c3,{v3},{v33});


%% acceleration max
A1=(2*a2)/(t2-t1)^2;
A2=(2*b2)/(t3-t2)^2;
A3=(2*c2)/(t4-t3)^2;
A4=(2*c2+6*c3)/(t4-t3)^2;

%% velocity max 
V1=(q2-q1)/(t2-t1); % 0 in the other case
V2=v22;
V3=v33;
V4=(q4-q3)/(t4-t3); % 0 in the other case

%% plot spline
pause;
tvals = [t1 t2 t3 t4];
qvals = [q1 q2 q3 q4];

splineplot(tvals,qvals)

