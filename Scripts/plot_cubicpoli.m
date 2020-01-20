%% Plot cubic polynomial

% time
T=3;
t=[0:0.1:T];

% cubic polynomial
q0=15; q1=45; dq=q1-q0;
v0=0; v1=0;
a=-1.5348; b=2.5348

% position
qp= q0+dq*(a*(t/T).^3 + b*(t/T).^2)

figure
hold on
plot(t,qp);grid; title('position');xlabel('time [s]');ylabel('[deg]')
pause;

% velocity
qv= dq*(3*a*(t/T).^2 + b*(t/T))

figure
hold on
plot(t,qv);grid; title('velocity');xlabel('time [s]');ylabel('[deg/s]')
pause;

% acceleration
qa=dq*(6*a*(t/T)+b)

figure
hold on
plot(t,qa);grid; title('acceleration');xlabel('time [s]');ylabel('[deg/s^2]')
pause;
