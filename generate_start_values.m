function result = generate_start_values(rc_i,point_coords)
    result(1) = rc_i(1);
    result(2) = rc_i(2);
    result(3) = 0;%get_body_orientation(get_global_vector_to_point(point_coords,rc_i));
end

