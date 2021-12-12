function skew_symmetric_matrix= S(vector)
    x = vector(1);
    y = vector(2);
    z = vector(3);
    skew_symmetric_matrix = [0 -z y; z 0 -x; -y x 0];
end

