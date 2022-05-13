function result = define_progressive_pair(body_i,point_i,body_j,point_j,fi_i_j,r_i,r_j,driving_func,driving_func_prim,driving_func_bis)
    result.body_i = body_i;
    result.body_j = body_j;
    result.point_i = point_i;
    result.point_j = point_j;
    result.fi_i_j = fi_i_j;
    temp_vec = r_j - r_i;
    result.driving_versor = (temp_vec) / norm(temp_vec);
    result.perpendicular_versor = Rot(pi/2) * result.driving_versor;
    result.driving_versor = Rot(pi)*result.driving_versor;
    
    if nargin < 10
        result.driving_func = '';
        result.driving_func_prim = '';
        result.driving_func_bis = '';
    else
        result.driving_func = driving_func;
        result.driving_func_prim = driving_func_prim;
        result.driving_func_bis = driving_func_bis;
    end
end
