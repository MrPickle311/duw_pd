function result = progressive_pair(ri,fi_i,sa_i,rj,fi_j,sb_j,fi_0,v_j)
    result(1) = fi_i - fi_j - fi_0; % 2.19
    result(2)=(Rot(fi_j)*v_j)'*...
    (rj+Rot(fi_j)*sb_j-ri-Rot(fi_i)*sa_i);     %wz. 2.20

end
