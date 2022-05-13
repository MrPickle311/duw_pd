function result = constrains_bis(q,q_prim,t,rot_pairs,prog_pairs,bodies,body0)

% kinematyczne
% obrotowe : wzór 2.42
% postępowe : pierwszy wiersz zerowy, drugi :wzór 2.59
% driving
% obrotowe: pierwszy wiersz: 
% postępowe : wzór 2.59 + druga_pochodna_func(t)

% pary obrotowe + ewentualne wiezy kierujace

rot_size = size(rot_pairs);
rot_size = rot_size(1);
prog_size = size(prog_pairs);
prog_size = prog_size(1);

rot_driving_pairs_count = 0;

for n = 1:rot_size
    i = rot_pairs(n).body_i;
    j = rot_pairs(n).body_j;
    point = rot_pairs(n).point;
    driving_func_bis = rot_pairs(n).driving_func_bis;
    
    [ri,fi_i,s_i,rj,fi_j,s_j] = get_current_data(q,bodies,body0,i,j,point,point);
    
    if i == 0
        fi_prim_i = 0;
    else
        fi_prim_i = q_prim(3*i,1);
    end
    
    fi_prim_j = q_prim(3*j,1);

    result(2*n-1:2*n,1) = rotate_acceleration(fi_i,s_i,fi_prim_i,fi_j,s_j,fi_prim_j);
    
    if ~strcmp(driving_func_bis,'')
        F(rot_size + prog_size + n,1) = driving_func_bis(t);
        rot_driving_pairs_count = rot_driving_pairs_count + 1;
    end
end

%pary postepowe + ewentualne wiezy kierujace

for n = 1:prog_size
    i = prog_pairs(n).body_i;
    j = prog_pairs(n).body_j;
    point_i = prog_pairs(n).point_i;
    point_j = prog_pairs(n).point_j;
    fi_i_j = prog_pairs(n).fi_i_j;
    perp = prog_pairs(n).perpendicular_versor;
    driv = prog_pairs(n).driving_versor;
    driving_func_bis = prog_pairs(n).driving_func_bis;
    
    [ri,fi_i,s_i,rj,fi_j,s_j] = get_current_data(q,bodies,body0,i,j,point_i,point_j);
    
    if i == 0
        fi_prim_i = 0;
        r_prim_i  = 0;
    else
        fi_prim_i = q_prim(3*i,1);
        r_prim_i  =  q_prim(3*i-2 : 3*i-1,1);
    end
    
    fi_prim_j = q_prim(3*j,1);
    r_prim_j  = q_prim(3*j-2 : 3*j-1,1);
    
    from = 2*rot_size+2*n - 1;
    to = from + 1;
    
    result(from:to,1) = progressive_acceleration(ri,r_prim_i,fi_i,fi_prim_i,s_i,...
                                      rj,r_prim_j,fi_j,fi_prim_j,perp);
    
    if ~strcmp(driving_func_bis,'')
        row = 2*rot_size + 2*prog_size + rot_driving_pairs_count + n;
        temp = progressive_acceleration(ri,r_prim_i,fi_i,fi_prim_i,s_i,...
                                      rj,r_prim_j,fi_j,fi_prim_j,driv) + driving_func_bis(t);
        result(row) = temp(2);
    end
end

end

