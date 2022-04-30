function [result] = insert_rotation_pair_into_jacobi(jacobi_matrix,body_i,body_j,row,fi_i,si,fi_j,sj)
    % niestety matlab przekazuje tylko przez wartosc, pass by value
    first_row = 2*row - 1;
    second_row = 2*row;
    temp = jacobi_element_for_rotation_pair(fi_i,si,fi_j,sj);
    result = jacobi_matrix;
    if body_i ~= 0
        result(first_row:second_row , 3*body_i - 2 : 3*body_i) = the_first_half_of_rotation_pair(temp);
    end
    result(first_row:second_row , 3*body_j - 2 : 3*body_j) = the_second_half_of_rotation_pair(temp);
end