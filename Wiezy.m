function F=Wiezy(q,t,rot_pairs,prog_pairs,bodies,body0)
% Rownania wiezow

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
    driving_func = rot_pairs(n).driving_func;
    
    [ri,fi_i,s_i,rj,fi_j,s_j] = get_current_data(q,bodies,body0,i,j,point,point);
    
    F(2*n-1:2*n,1) = rotate_pair(ri,fi_i,s_i,rj,fi_j,s_j);
    
    if ~strcmp(driving_func,'')
        F(rot_size + prog_size + n,1) = fi_i - fi_j - driving_func(t);
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
    driving_func = prog_pairs(n).driving_func;
    
    [ri,fi_i,s_i,rj,fi_j,s_j] = get_current_data(q,bodies,body0,i,j,point_i,point_j);
    
    from = 2*rot_size+2*n - 1;
    to = from + 1;
    
    F(from:to,1) = progressive_pair(ri,fi_i,s_i,rj,fi_j,s_j,fi_i_j,perp);
    
    if ~strcmp(driving_func,'')
        row = 2*rot_size + 2*prog_size + rot_driving_pairs_count + n;
        temp = progressive_pair(ri,fi_i,s_i,...
            rj,fi_j,s_j,fi_i_j,driv) - driving_func(t);
        F(row,1) = temp(2);
    end
end
end
