function result = relative_displacement_in_progressive_pair(ri,fi_i,sa_i,rj,fi_j,sb_j,u_j,func,t)
    result = ( Rot(fi_j)*u_j )' * ( rj + Rot(fi_j)*sb_j - ri - Rot(fi_i)*sa_i ) - func(t);
end

