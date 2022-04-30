function result = relative_orientation_two_bodies(fi_i,fi_j,t,Func_j_i)
    result = fi_i - fi_j - Func_j_i(t);
end

