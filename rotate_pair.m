function result = rotate_pair(ri,fi_i,sa_i,rj,fi_j,sb_j)
    result = ri + Rot(fi_i)*sa_i - ( rj + Rot(fi_j)*sb_j );
end

