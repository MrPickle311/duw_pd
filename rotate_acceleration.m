function result = rotate_acceleration(fi_i,sa_i,fi_prim_i,fi_j,sb_j,fi_prim_j)
    result = Rot(fi_i)*sa_i*fi_prim_i*fi_prim_i - ...
        Rot(fi_j)*sb_j*fi_prim_j*fi_prim_j;
end

