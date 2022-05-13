function [result] = insert_progressive_driving_pair_into_jacobi(jacobi_matrix,body_i,body_j,row,ri,fi_i,s_i,rj,fi_j,vec)
    % niestety matlab przekazuje tylko przez wartosc, pass by value
    first_row = row;
    temp = jacobi_element_for_progressive_pair(ri,fi_i,s_i,rj,fi_j,vec);
    result = jacobi_matrix;
    
    first_half = the_first_half_of_constrain_pair(temp)
    second_half = the_second_half_of_constrain_pair(temp)
    first_half(2,:)
    second_half(2,:)
    
    if body_i ~= 0
        result(first_row , 3*body_i - 2 : 3*body_i) = first_half(2,:);
    end
    result(first_row , 3*body_j - 2 : 3*body_j) = second_half(2,:);
end