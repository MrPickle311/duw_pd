function result = constrain_first_dot(q,t,rot_pairs,prog_pairs,bodies,body0)

% tutaj niezerowe sa tylko pochodne funkcji z wiezow kierujacych

rot_size = size(rot_pairs);
rot_size = rot_size(1);
prog_size = size(prog_pairs);
prog_size = prog_size(1);
rot_driving_pairs_count = 0;

for n = 1:rot_size
    driving_func_prim = rot_pairs(n).driving_func_prim;
   
    if ~strcmp(driving_func_prim,'')
        result(rot_size + prog_size + n,1) = driving_func_prim(t);
        rot_driving_pairs_count = rot_driving_pairs_count + 1;
    end
end

for n = 1:prog_size
    driving_func_prim = prog_pairs(n).driving_func_prim;
    
    if ~strcmp(driving_func_prim,'')
        row = 2*rot_size + 2*prog_size + rot_driving_pairs_count + n;
        result(row,1) = driving_func_prim(t);
    end
end

end

