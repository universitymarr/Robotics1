function [Jl, Ja] = geometric_jacobian(f_r, sequence, q_in, params)
% geoJ = geometric_jacobian() takes as inputs:
%   -f_r: mapping from joint space to cartesian space
%   -sequence: A string containing the sequence of 'r's and 'p's
%   -q_in: The list of the variables we want to derive wrt
%   -params: List of params needed to get the DHMatrix
% and outputs:
%   -geoJ: The resulting geometric jacobian

    [~, A] = DHMatrix(params);
    sequence = char(lower(sequence));
    n_dof = strlength(sequence);
    
    f_r = f_r(1:3, :);
    
    %to calculate Jl, we use analytic jacobian
    Jl = jacobian(f_r, q_in);
    
    Ja = sym(zeros(3, n_dof));
    
    cells = cell(1, n_dof);
    cells{1} = get_rotation_mat(A{1});
    for i = 2:n_dof
        cells{i} = cells{i-1}*get_rotation_mat(A{i});
    end
    
    for i = 1:n_dof
        c = sequence(i);
        
        if c ~= 'r' && c ~= 'p'
            disp("Sequence must be composed of only 'r's and 'p's")
            geoJ = -1;
            return
        end
        
        %if the joint is prismatic then the element in Ja is equals to zero
        %otherwise:
        %   Ja(:, i) = z_(i-1) = R0_1 * ... * R(i-2)_(i-1)*[0; 0; 1]
        if c == 'r'
            if i == 1
                Ja(:, i) = [0; 0; 1];
            else
                Ja(:, i) = cells{i-1}*[0; 0; 1];
            end
        end
    end
    Jl = simplify(Jl);
    Ja = simplify(Ja);
    %geoJ = simplify([Jl; Ja]);

end