%to call a newton/gradient method and plot the cartesian error
syms q1 q2 q3 q4 q5 q6;

%all the parameters of the robot
d = 0.5;

%all the desired/actual configuration + direct kinematics
q_in = [q1; q2];
desired_point = [1; 1];
f_r = [q2*cos(q1); q2*sin(q1)];
initial_guess = [pi/4; 10^(-9)];

max_iterations = 15;
max_cartesian_error = 10^(-5);
min_joint_increment = 10^(-6);
max_closeness_singularity = 10^(-4);

%Newton method
%[q_out, guesses, cartesian_errors] = newton_method(q_in, desired_point, f_r, initial_guess, max_iterations, max_cartesian_error, min_joint_increment, max_closeness_singularity)

%if you want to use the gradient method you have also to specify an
%alpha: The "learning rate" of the algorithm
alpha = 1;
[q_out, guesses, cartesian_errors] = gradient_method(q_in, desired_point, f_r, initial_guess, alpha, max_iterations, max_cartesian_error, min_joint_increment, max_closeness_singularity)

%if you want to plot them
names = ['1' '2' '3'];
figure
hold on
plot(guesses);grid; title('joint variables evolution');xlabel('iterations');ylabel('q[rad]')
legend(arrayfun( @num2str,  names, 'UniformOutput', false ))
hold off
legend show

figure
hold on
plot(cartesian_errors);grid; title('cartesian errors');xlabel('iterations');ylabel('error norm')
