function result = progressive_acceleration(r_i,r_prim_i,fi_i,fi_prim_i,sa_i,r_j,r_prim_j,fi_j,fi_prim_j,v_j)
Om=[0 -1;
    1 0];
% wzor 2.59

result(1) = 0;

result(2) = (Rot(fi_j)*v_j)' * ...
    (2*Om*(r_prim_j - r_prim_i)*fi_prim_j + ...
     (r_j - r_i)*fi_prim_j^2 - ...
      Rot(fi_i)*sa_i*(fi_prim_j-fi_prim_i)^2);
end