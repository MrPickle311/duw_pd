function result = get_vector_from_local_origin_to_point(point_coordinates, body_coordinates)
    global_vector = point_coordinates - body_coordinates;
    result = Rot(get_body_orientation(global_vector))' * global_vector;
end

