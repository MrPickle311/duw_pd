function result = produce_body0(points_descriptors , global_vectors_to_points) 
    for i = 1:strlength(points_descriptors)
        result{i} = produce_body_entity(points_descriptors(i) ,...
                                        global_vectors_to_points(:,i));
                                    result{i}.point
                                    result{i}.local_vec
    end
end