function result=Jakobian(q,rot_pairs,prog_pairs,bodies,body0)

% Macierz Jaocobiego - początkowo zerowa
result=zeros(30,30);

% Macierz Jaocobiego - wypelnianie niezerowych elementów

% pary obrotowe

rot_size = size(rot_pairs);
rot_size = rot_size(1);
prog_size = size(prog_pairs);
prog_size = prog_size(1);
rot_driving_pairs_count = 0;

for n = 1:rot_size
    i = rot_pairs(n).body_i;
    j = rot_pairs(n).body_j;
    point = rot_pairs(n).point;
    
    [ri,fi_i,s_i,rj,fi_j,s_j] = get_current_data(q,bodies,body0,i,j,point,point);

    result = insert_rotation_pair_into_jacobi(result,i,j,n,fi_i,s_i,fi_j,s_j);
    
    if rot_pairs(n).driving_func ~= ''
        
        rot_driving_pairs_count = rot_driving_pairs_count + 1;
        row = rot_size + prog_size + n;
        
        if i > 0
            result(row, 3*i) = 1; 
        end
        
        if j > 0
            result(row, 3*j) = -1;
        end
    end
end

% pary postepowe

for n = 1:prog_size
    i = prog_pairs(n).body_i;
    j = prog_pairs(n).body_j;
    point_i = prog_pairs(n).point_i;
    point_j = prog_pairs(n).point_j;
    perp = prog_pairs(n).perpendicular_versor;
    driv = prog_pairs(n).driving_versor;
    driving_func = prog_pairs(n).driving_func;
    
    [ri,fi_i,s_i,rj,fi_j,s_j] = get_current_data(q,bodies,body0,i,j,point_i,point_j);

    current_row = 2*rot_size+2*n - 1;
    
    result = insert_progressive_pair(result,i,j,current_row,ri,fi_i,s_i,rj,fi_j,perp);
    
    if ~strcmp(driving_func,'')
        row = 2*rot_size + 2*prog_size + rot_driving_pairs_count + n;
        result = insert_progressive_driving_pair_into_jacobi(result,i,j,row,ri,fi_i,s_i,rj,fi_j,driv);
    end
end

epsilon = 1e-6;
if (cond(result) > (1/epsilon()))
    error('Macierz Jacobiego osobliwa');
end


end
