function out=diff_print(exp, joints, n_level_diff, mode)
    if nargin < 4
        mode = 'direct';
    end
    
    n = length(joints);
    
    q_diff = 'q';
    
    to_remove = [];
    replace = [];
    syms t;
    for i=1:n_level_diff
        q_diff = strcat('d', q_diff);
        for j=1:n    
            q_diff_j = strcat(q_diff, int2str(j)); 
            to_remove = [to_remove, diff(joints(j),t,i)];
            replace = [replace, str2sym(q_diff_j)];
        end
    end
    
    if strcmp(mode, 'reverse') 
        out = subs(exp, replace, to_remove)
    else
        out = subs(exp, to_remove, replace)
    end 
end
