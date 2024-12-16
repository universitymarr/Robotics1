function [Ja] = DH_to_JA(DHTABLE, prismatic_indices)
    %Function that find the angular jacobian for a robot 
    %
    %parameters:
    %-DH in the order alpha a d theta, ex:
    %   DHTABLE = [ pi/2    0       sym('d1')   q1;
    %               pi/2    0           0       q2;
    %               pi/2    0          q3       0;
    %               0      sym('a4')    0       q4];
    %-prismatic_indices: a list of indices of prismatic Joints ex: [2,4]
    %   if there is no prismatic joint put []
    syms alpha_ d a_ theta_

    N = size(DHTABLE, 1);

    TDH = [cos(theta_) -sin(theta_)*cos(alpha_) sin(theta_)*sin(alpha_) a_*cos(theta_);
           sin(theta_) cos(theta_)*cos(alpha_) -cos(theta_)*sin(alpha_) a_*sin(theta_);
           0 sin(alpha_) cos(alpha_) d;
           0 0 0 1];

    A = cell(1, N);

    
    for i = 1:N
        A{i} = subs(TDH, [alpha_, a_, d, theta_], DHTABLE(i, :));
    end

    T = eye(4);
    rotation_0_to_i = cell(1, N);

    for i = 1:N
        T = T * A{i};
        rotation_0_to_i{i} = simplify(T);
    end

    % Initialize the Jacobian matrix as symbolic
    Ja = sym(zeros(3, N));

    % Unit vector along the z-axis
    z = [0; 0; 1];
    Ja(:, 1) = z;
    % Compute the column vectors for the Jacobian matrix
    for i = 1:N-1
        R = rotation_0_to_i{i}(1:3, 1:3);
        zi = R * z;
        Ja(:, i+1) = zi;
    end

    % Substitute columns with zeros for prismatic joints
    for prismatic_index = prismatic_indices
        if prismatic_index ~= N
            Ja(:, prismatic_index) = zeros(3, 1);
        end
    end

    % Display or return the Jacobian matrix as needed
    disp('Angular Jacobian Matrix:');
    disp(Ja);
end

