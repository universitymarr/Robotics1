function q_dot = SNS(x_dot, J, Q_min_dot, Q_max_dot, print_info)

%{
Q_min_dot = [-2.8, -3.6, -4]'
Q_max_dot = -Q_min_dot
x_dot = [4, 2]'
print_info = true
%}


[m, n] = size(J);
W = eye(n);
q_N_dot = zeros(n, 1);
q_dot = zeros(n, 1);

s=1;
s_star = 0;
W_star = W;
q_N_dot_star = q_N_dot; 

if print_info
    fprintf("########################## SNS ################################\n");
end
it = 0;
while 1
    it = it + 1;
    limit_exceeded = false;
    q_dot = q_N_dot + pinv(J*W)*(x_dot-J*q_N_dot);

    if print_info
        fprintf("Iteration %d --------------------------------------------------\n", it);
        fprintf("Current solution (q_dot = q_N_dot + pinv(J*W)*(x_dot-J*q_N_dot):\n\t");
        display(q_dot);
    end
    
    if violated_bond(q_dot, Q_min_dot, Q_max_dot)
        limit_exceeded = true;
        a = pinv(J*W)*x_dot;
        b = q_dot - a;
        
        [critical_joint, task_scaling_factor] = getTaskScalingFactor(n, Q_min_dot, Q_max_dot, a, b);
        %display([critical_joint, task_scaling_factor])
        
        if print_info
            fprintf("Bound of joint %d violated!\n", critical_joint);
        end
        
        if task_scaling_factor > s_star
            s_star = task_scaling_factor;
            W_star = W;
            q_N_dot_star = q_N_dot;
        end
        
        j = critical_joint;
        W(j, j) = 0;
        %display(W);
        if q_dot(j) > Q_max_dot(j)
            q_N_dot(j) = Q_max_dot(j);
        elseif q_dot(j) < Q_min_dot(j)
            q_N_dot(j) = Q_min_dot(j);
        end
        
        if print_info
            fprintf("Saturated joint %d\n", critical_joint);
            fprintf("\tq_%d_dot = %f\n", j, q_N_dot(j));
        end
        
        if rank(J*W) < m
            %display(rank(J*W))
            s = s_star;
            W = W_star;
            q_N_dot = q_N_dot_star;
            q_dot = q_N_dot + pinv(J*W)*(s*x_dot-J*q_N_dot);
            if print_info
                fprintf("rank(J*W) < m\n\t");
                fprintf("New scaled solution (q_dot = q_N_dot + pinv(J*W)*(s*x_dot-J*q_N_dot):\n\t");
                fprintf("\t(task_scaling_factor chosen is s=%f)", task_scaling_factor);
                display(q_dot);
            end
            limit_exceeded = false;
        else
            if print_info
                fprintf("rank(J*W) >= m\n");
                fprintf("\tContinue to iterate ...\n");
            end
        end
        
    end
    
    if print_info
        fprintf("--------------------------------------------------------------\n\n");
    end
    if limit_exceeded == false
        if print_info
            fprintf("SNS Algorith terminated\n");
            fprintf("Solution: \n");
            display(q_dot);
            fprintf("Norm solution: %f\n", norm(q_dot));
            fprintf("##########################################################");
        end
        break;
    end
end
end



function [critical_joint, task_scaling_factor] = getTaskScalingFactor(N, Q_min_dot, Q_max_dot, a, b)
    S_min = zeros(N, 1);
    S_max = zeros(N, 1);
    
    for i = 1:N
        s_min__ = (Q_min_dot(i)-b(i)) / a(i);
        s_max__ = (Q_max_dot(i)-b(i)) / a(i);
        
        if s_min__ > s_max__
            S_min(i) = s_max__;
            S_max(i) = s_min__;
        else
            S_min(i) = s_min__;
            S_max(i) = s_max__;
        end
    end
    
    %display(S_max)
    [M,I] = min(S_max);
    s_max = M;
    s_min = max(S_min);
    
    critical_joint = I; 
    
    if s_min > s_max || s_max < 0 || s_min > 1
        task_scaling_factor = 0;
    else
        task_scaling_factor = s_max;
    end
    
    %display(critical_joint)
end

function res = violated_bond(q_dot, Q_min_dot, Q_max_dot)
    res = false;
    [r, c] = size(q_dot);
    n = r;
    for i = 1:n
        if q_dot(i) < Q_min_dot(i) || q_dot(i) > Q_max_dot(i)
            res = true;
            return 
        end
    end
end

