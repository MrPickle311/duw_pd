function result = jacobi_element_for_rotation_pair(fi_i,sa_i,fi_j,sb_j)
    Om=[0 -1;
        1 0];
    result = zeros(2,6);
    
    result(1:2,1:2) = eye(2);
    result(1:2,3) = Om*Rot(fi_i)*sa_i;
    result(1:2,4:5) = -eye(2);
    result(1:2,6) = -Om*Rot(fi_j)*sb_j;
end

