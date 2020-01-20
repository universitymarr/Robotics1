function T = homogeneous(R, P)
% T = homogeneous(R, P) takes as inputs:
%   -R: A rotation matrix
%   -P: A translation/position vector
% and outputs:
%   -T: The resulting roto-translation homogeneous matrix

    T = [
      R(1, :) P(1);
      R(2, :) P(2);
      R(3, :) P(3);
      0 0 0 1   
    ];
        
end