function result = get_local_vector_from_body_0(body_0,point)
    s = size(body_0);
    for k = 1:s(2)
        if strcmp(point,body_0{k}.point)
            result = body_0{k}.local_vec;
            return
        end
    end
end

