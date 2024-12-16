
%% Plot trigonometric

T=3;
t=[0:0.1:T];

%% trigonometric 
a1=15 a2=-5 b1=-15 b2=-5

% position
qp=a1*sin(t/T)+a2*sin((3*pi/2)*(t/T))+b1*cos((pi/2)*(t/T))+b2*cos((3*pi/2)*(t/T))

figure
hold on
plot(t,qp);grid; title('position');xlabel('time [s]');ylabel('[deg]')
pause;

% velocity
qv=(1/T)*a1*cos(t/T)+a2*(3*pi/(2*T))*cos((3*pi/2)*(t/T))-b1*(pi/(2*T))*sin((pi/2)*(t/T))-b2*(3*pi/(2*T))*sin((3*pi/2)*(t/T))

figure
hold on
plot(t,qv);grid; title('velocity');xlabel('[s]');ylabel('[rad/s]')
pause;

% acceleration
qa=-(1/T)^2*a1*sin(t/T)-a2*(3*pi/(2*T))^2*sin((3*pi/2)*(t/T))-b1*(pi/(2*T))^2*cos((pi/2)*(t/T))-b2*(3*pi/(2*T))^2*cos((3*pi/2)*(t/T))

figure
hold on
plot(t,qa);grid; title('velocity');xlabel('[s]');ylabel('[rad/s]')
pause;
