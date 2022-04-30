function result = get_local_vector(bodies,body_nmbr,point)
    body = bodies{body_nmbr};
    s = size(bodies{2});
    for k = 1:s(2)
        if strcmp(point,body{k}.point)
            result = body{k}.local_vec;
            return
        end
    end
end

