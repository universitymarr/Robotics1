function out=sc_print(exp, joints, mode)
    if nargin < 3
        mode = 'direct'; 
    end
    n = length(joints);
    sin_array = sym('S%d', [1,n]);
    cos_array = sym('C%d', [1,n]);
    
    to_remove = [];
    replace = [];
    
    for i=1:n
        to_remove = [to_remove, sin(joints(i)), cos(joints(i))];
        replace = [replace, sin_array(i), cos_array(i)];
    end
    
    if strcmp(mode, 'reverse') 
        out = subs(exp, replace, to_remove)
    else
        out = subs(exp, to_remove, replace)
    end   
end
