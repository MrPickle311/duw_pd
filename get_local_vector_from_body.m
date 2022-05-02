function result = get_local_vector_from_body(body, point_descriptor)
    body_size = size(body);
    for k = 1:body_size(2)
        if strcmp(point_descriptor,body{k}.point)
            result = body{k}.local_vec;
            return
        end
    end
end

