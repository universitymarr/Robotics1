function print_cells(celle)
    for i=1:size(celle,2)
        fprintf('cell{%d} = \n', i)
        disp(celle{i})
    end
end