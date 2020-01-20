
%% Plot quintic polynomial

T=3;
t=[0:0.1:T];

% quintic polynomial
q0=[0.8697; 1.2132;]; qf=[2.8391;1.7824]; dq=qf-q0;
con v0,v1,a0,a1 =0;

% position
q1t=q0(1)+dq(1)*(6*(t/T).^5 -15*(t/T).^4+10*(t/T).^3)
q2t=q0(2)+dq(2)*(6*(t/T).^5 -15*(t/T).^4+10*(t/T).^3)

figure
hold on
plot(t,q1t);grid; title('position');xlabel('time [s]');ylabel('[deg]')
plot(t,q2t)
pause;

% velocity
q1v=dq(1)*((30/T)*(t/T).^4 -(60/T)*(t/T).^3+(30/T)*(t/T).^2)
q2v=dq(2)*((30/T)*(t/T).^4 -(60/T)*(t/T).^3+(30/T)*(t/T).^2

figure
hold on
plot(t,q1v);grid; title('velocity');xlabel('[s]');ylabel('[rad/s]')
plot(t,q2v)
pause;

% acceleration
q1a=dq(1)*((120/T^2)*(t/T).^3 -(180/T^2)*(t/T).^2+(60/T^2)*(t/T))
q2a=dq(2)*((120/T^2)*(t/T).^3 -(180/T^2)*(t/T).^2+(60/T^2)*(t/T))
pause;

figure
hold on
plot(t,q1a);grid; title('accelaration');xlabel('[s]');ylabel('[rad/s^2]')
plot(t,q2a);grid; title('accelaration');xlabel('[s]');ylabel('[rad/s^2]')
