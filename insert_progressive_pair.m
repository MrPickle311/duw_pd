function [result] = insert_progressive_pair(jacobi_matrix,body_i,body_j,row,ri,fi_i,s_i,rj,fi_j,vec)
    % niestety matlab przekazuje tylko przez wartosc, pass by value
    first_row = row;
    second_row = row + 1;
    temp = jacobi_element_for_progressive_pair(ri,fi_i,s_i,rj,fi_j,vec);
    result = jacobi_matrix;
    if body_i ~= 0
        result(first_row:second_row , 3*body_i - 2 : 3*body_i) = the_first_half_of_constrain_pair(temp);
    end
    result(first_row:second_row , 3*body_j - 2 : 3*body_j) = the_second_half_of_constrain_pair(temp);
end

