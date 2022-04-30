function [result] = insert_rotation_pair_into_jacobi(jacobi_matrix,body_i,body_j,row,fi_i,si,fi_j,sj)
    % niestety matlab przekazuje tylko przez wartosc, pass by value
    temp = jacobi_element_for_rotation_pair(fi_i,si,fi_j,sj);
    result = jacobi_matrix;
    if body_i ~= 0
        result(row:row+1,body_i:body_i+2) = the_first_half_of_rotation_pair(temp);
        result(row:row+1,3*body_j-2:3*body_j) = the_second_half_of_rotation_pair(temp);
    else
        result(row:row+1,3*body_j-2:3*body_j) = the_second_half_of_rotation_pair(temp);
    end
end