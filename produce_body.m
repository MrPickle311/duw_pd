function result = produce_body(points_descriptors , global_vectors_to_points, rc_i) 
    % wazne jest zachowanie takiej samej kolejnosci punktow w obu argumentach
    for i = 1:strlength(points_descriptors)
        result{i} = produce_body_entity(points_descriptors(i) ,...
           get_vector_from_local_origin_to_point(global_vectors_to_points(:,i),rc_i));
    end
end