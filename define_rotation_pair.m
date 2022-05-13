function result = define_rotation_pair(body_i,body_j,point,driving_func,driving_func_prim,driving_func_bis)
    result.body_i = body_i;
    result.body_j = body_j;
    result.point = point;
    
    if nargin < 6
        result.driving_func = '';
        result.driving_func_prim = '';
        result.driving_func_bis = '';
    else
        result.driving_func = driving_func;
        result.driving_func_prim = driving_func_prim;
        result.driving_func_bis = driving_func_bis;
    end
end