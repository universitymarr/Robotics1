function T = affine_T(rotation_matrix, translation)
    T = [rotation_matrix, translation; 0 0 0 1];
end

