function res = isRotationMatrix(R)
    RR = eval(simplify(R'*R));
    det_R = eval(simplify(det(R)));
    if isequal(RR, eye(3)) && isequal(det_R, 1)
        res = true;
    else
        res = false;
    end
end

