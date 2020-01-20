function [T, A] = DHMatrix(arrays)
% T = DHMatrix(arrays) takes as inputs:
%   -arrays: a n-vector of vectors composed like this: [alpha a d theta]
% and outputs:
%   -T: the product of all the matrices corresponding to each vector of arrays
% Remember that:
% cos(q1 + q2) = cos(q1)*cos(q2) - sin(q1)*sin(q2)
% sin(q1 + q2) = cos(q1)*sin(q2) + cos(q2)*sin(q1)
% making use of the simplify function these are converted automatically

    T = eye(4);
    nums = size(arrays);
    
    A = cell(1,nums(1));
    
    for i = 1:nums(1)
        line = arrays(i, :);
        R = [cos(line(4)) -cos(line(1))*sin(line(4)) sin(line(1))*sin(line(4)) line(2)*cos(line(4));
             sin(line(4)) cos(line(1))*cos(line(4)) -sin(line(1))*cos(line(4)) line(2)*sin(line(4));
             0 sin(line(1)) cos(line(1)) line(3);
             0 0 0 1;];
        A{i} = R;
        T = T * R;   
    end

    if isa(T, 'sym')
        T = simplify(T);
    end
end