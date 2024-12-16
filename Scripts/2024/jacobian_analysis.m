function jacobian_analysis(J, variables)
    %performs the full analysis of a Jacobian, square or not,
    %displaying: 
    %-determinant
    %-singularity
    %-singular Jacobian--x
    %-rank
    %-nullspace
    %-nullspace in the singular jacobian --x
    %-range
    %
    %parameters:
    % J = jacobian matrix
    % v = list of symbolic variables (only the one used to differentiate
    % the Jacobian, don't put constants in there)

    % Check if the jacobian is squared
    disp("-------------------------------------------------------------------------------")
    disp("Jacobian Analysis (Jacobian, Determinant, Singularity, Rank, Nullspace, Range)")
    disp("-------------------------------------------------------------------------------")
    [m, n] = size(J);
    if m ~= n % If m not equal to n (joints) | If m == n, then it's square.
        if n - m >= 2
            % Compute the determinant
            determinant_Jacobian = simplify(det(J.' * J));
        
            disp('Determinant of the Jacobian (J.T * J) --> Since its not square:');
            disp(simplify(determinant_Jacobian));
        
            disp("You might want to check when the Jacobian loses full rank.")
            disp("For doing so, you have to solve the Determinant above for the values which make it == 0.")
            disp("Example: solve(simplify(det(J.' * J)) == 0, variables);")
        elseif m < n
            for i = 1:n
                % Create a submatrix of the original non-square matrix by
                % removing one column at a time
                Jcopy=J;
                Jcopy(:,i)=[];
    
                % Determinant of the i-th submatrix
                fprintf('Determinant of the submatrix with column %d removed: \n', i);
                disp(simplify(det(Jcopy)));
    
                % Compute the singularity (equal the determinant to zero)
                fprintf('Singularity condition of the %s -th submatrix: \n', mat2str(i));
                disp(simplify(det(Jcopy)==0));
            end
        else 
            %create all submatrix generated from removing any m-n rows

            %case of Jacobian 6*4
            if m==6 & n==4
                %create the sumatrices by creating all the 4by4
                %submatrices
                k=0;
                for i = 1:m
                    % Create a submatrix of the original non-square matrix by
                    % removing one column at a time
                    Jcopy=J;
                    Jcopy(i,:)=[];
                    for j=1:m-1
                        k=k+1;
                        Jcopy2=Jcopy;
                        Jcopy2(j,:)=[];
                        % Determinant of the i-th submatrix
                        fprintf('Determinant of the submatrix with column %d removed: \n', i);
                        disp(simplify(det(Jcopy2)));
                    end
                end
            else
                disp('The jacobian you have parsed has more rows than columns and its not a geometric jacobian')
                % Compute the determinant
                determinant_Jacobian = simplify(det(J.' * J));
        
                disp('Determinant of the Jacobian:');
                disp(simplify(determinant_Jacobian));
        
                disp("You might want to check when the Jacobian loses full rank.")
                disp("For doing so, you have to solve the Determinant above for the values which make it == 0.")
                disp("Example: solve(simplify(det(J.' * J)) == 0, variables);")
            end
        end
    else
        % Compute the determinant
        determinant_Jacobian = simplify(det(J));

        disp('Determinant of the Jacobian:');
        disp(simplify(determinant_Jacobian));

        n_var = length(variables);
        % Solve for all possible values of the symbols that make the determinant zero
        if n_var == 2
            [q1_sol, q2_sol, ~, ~] = solve(determinant_Jacobian == 0, variables, 'ReturnConditions', true);
            solutions = [q1_sol, q2_sol];
        elseif n_var == 3
            [q1_sol, q2_sol, q3_sol, ~, ~] = solve(determinant_Jacobian == 0, variables, 'ReturnConditions', true);
            solutions = [q1_sol, q2_sol, q3_sol];
        elseif n_var == 4
            [q1_sol, q2_sol, q3_sol, q4_sol, ~, ~] = solve(determinant_Jacobian == 0, variables, 'ReturnConditions', true);
            solutions = [q1_sol, q2_sol, q3_sol, q4_sol];
        end
        disp('All possible solutions for the symbols that make the determinant zero:');
        display(solutions);
    end
        % Compute the rank of the Jacobian
        rank_J = rank(J);
        
        % Display the rank of the Jacobian
        disp('Rank of the Jacobian:');
        disp(rank_J);
        
        % Compute the nullspace of the Jacobian
        nullspace_J = null(J);
        
        % Display the nullspace of the Jacobian
        disp('Nullspace of the Jacobian:');
        disp(simplify(nullspace_J));

        % Compute the range of the jacobian
        range = simplify(orth(sym(J)));

        % Display the range
        disp('Range of the jacobian:');
        disp(range);

        disp('I will display the basis of the range now. -- Warning! this result might be wrong, please check by hand if possible')
        % Display the basis of the range
        [~,m]=size(range);
        for i=1:m
            [numerator, ~] = numden(range(:,i));
            fprintf('%d basis',i)
            display(simplify(numerator,'Steps',50));
        end

        % Compute the complementary nullspace of the Jacobian
        nullspace_J_complementary = null(J');
        if isempty(nullspace_J_complementary)
            disp('The complementary nullspace is empty')
        else
            % Display the complementary nullspace of the Jacobian
            disp('Nullspace of the Jacobian (complementary):');
            disp(simplify(nullspace_J_complementary));
        end
        disp("-------------------------------------------------------------------------------")
end