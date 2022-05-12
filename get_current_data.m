function [ri,fi_i,s_i,rj,fi_j,s_j] = get_current_data(q,bodies,body0,i,j,point_i,point_j)
	r0=[0;0];       
    fi0 = 0;
    if i == 0
        ri = 0;
        fi_i = fi0;
        s_i = get_local_vector_from_body_0(body0,point_i);
    else
        ri = q(3*i-2 : 3*i-1,1);
        fi_i = q(3*i,1);
        s_i = get_local_vector(bodies,i,point_i);
    end

    rj = q(3*j-2 : 3*j-1,1);
    fi_j = q(3*j,1);
    s_j = get_local_vector(bodies,j,point_j);
end

