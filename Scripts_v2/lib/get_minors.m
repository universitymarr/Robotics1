function cells_minors=get_minors(matrix, size_minor_matrix)
    cells_minors={};
    selector_code_rows = get_minors__get_selector_code(size(matrix,1), size_minor_matrix);
    selector_code_cols = get_minors__get_selector_code(size(matrix,2), size_minor_matrix);
    k=1;
    for i =1:size(selector_code_rows,1)
        for j =1:size(selector_code_cols,1)
            selector_code_rows(i,:);
            selector_code_cols(j,:);
            minor = get_minors__remove_rows_cols(matrix, selector_code_rows(i,:), selector_code_cols(j,:));
            cells_minors{k} = minor;
            k = k+1;
        end
    end
end

function reduced_matrix = get_minors__remove_rows_cols(matrix, selector_code_rows, selector_code_cols)
    reduced_matrix = [];
    %remove rows
    selec=[];
    for i=1:size(selector_code_rows,2)
        if selector_code_rows(i) == 0
            selec = [selec i];
        end
    end
    reduced_matrix=matrix(selec,:);
    %remove cols
    selec=[];
    for i=1:size(selector_code_cols,2)
        if selector_code_cols(i) == 0
            selec = [selec i];
        end
    end
    reduced_matrix=reduced_matrix(:,selec);
end

function selector_code = get_minors__get_selector_code(one_dim_matrix, one_dim_minors)

    diff_rows = one_dim_matrix-one_dim_minors;
    selector_code = [];
    input = 0:(2^one_dim_matrix-1);
    size_word = one_dim_matrix;
    b_array = de2bi_(input,size_word,2,'left-msb');
    
    for i=1:size(b_array,1)
        if sum(b_array(i,:)) == diff_rows
            selector_code = [selector_code; b_array(i,:)];
        end
    end
end

function b = de2bi_(d, n, p, flg)
    % DE2BI converts decimal numbers to their binary representation
    %
    % This is a drop-in replacement for the corresponding function from
    % the Mathworks Communications System Toolbox.
    
    % see http://se.mathworks.com/help/releases/R2015b/comm/ref/de2bi.html
    
    d = d(:);
    m = numel(d);
    
    if nargin<2 || isempty(n)
      n = floor(log(max(d))/log(2)+1);
    end
    
    if nargin>=3 && p~=2
      % only supported for base 2
      error('not implemented');
    end
    
    if nargin<4 || isempty(flg)
      flg = 'right-msb';
    end
    
    b = zeros(m,n);
    for i=1:m
      b(i,:) = bitget(d(i),1:n);
    end
    
    switch flg
      case 'left-msb'
        % flip left-right
        b = fliplr(b);
      case 'right-msb'
        % keep as it is
      otherwise
        error('not implemented');
    end
end