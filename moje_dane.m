function [rot_pairs,prog_pairs,body_0,bodies,q0] = moje_dane()
    % konfiguracja poczatkowa w ukladzie globalnym
    % z pdfa

    rA0 = [1.5 -0.6]';
    rB0 = [1.3 0.0]';
    rC0 = [0.5 0.4]';
    rD0 = [0.8 0.0]';
    rE0 = [0.5 0.2]';
    rF0 = [-0.3 0.3]';
    rG0 = [0.3 0.3]';
    rH0 = [0.1 -0.7]';
    rK0 = [1.9 -0.5]';
    rM0 = [0.8 -0.2]';
    rN0 = [0.0 -0.4]';
    rO0 = [0.0 0.0]';
    rP0 = [-0.2 0.1]';

    % wspolrzedne srodkow mas czlonow w ukladzie globalnym 
    % w konfiguracji poczatkowej
    % z pdfa
    rc10 = [0.7 -0.2]';
    rc20 = [0.0 0.2]';
    rc30 = [0.2 0.3]';
    rc40 = [1.55 -0.35]';
    rc50 = [0.9 0.2]';
    rc60 = [0.2 -0.35]';
    rc70 = [0.6 -0.25]';
    rc80 = [0.15 -0.45]';
    rc90 = [0.25 0.05]';
    rc100 = [0.7 0]';

    % definicja par obrotowych + ewentualne funkcje wymuszajace ruch
    
    rot_pairs = [
                 define_rotation_pair(1,4,'A'); % np. para 1-4 punkt A
                 define_rotation_pair(4,5,'B');
                 define_rotation_pair(3,5,'C');
                 define_rotation_pair(1,10,'D');
                 define_rotation_pair(3,10,'E');
                 define_rotation_pair(2,3,'F');
                 define_rotation_pair(2,9,'G');
                 define_rotation_pair(0,8,'H');
                 define_rotation_pair(7,10,'M');
                 define_rotation_pair(0,6,'N');
                 define_rotation_pair(0,2,'O');
                 define_rotation_pair(1,2,'P');
                 ];

             
    % definicja par postępowych + ewentualne funkcje wymuszajace ruch
    
    lk1 = sqrt(17)/5;
    ak1 = 0.1;
    wk1 = 0.1;
    fi_k1 = 0.0;
    
    lk2 = sqrt(26)/5;
    ak2 = 0.1;
    wk2 = 0.1;
    fi_k2 = 0.0;
    
    func1 = @(t_i) lk1 + ak1*sin(wk1*t_i + fi_k1);
    func2 = @(t_i) lk2 + ak2*sin(wk2*t_i + fi_k2);
    
    func1_prim = @(t_i) ak1*wk1*cos(t_i*wk1+fi_k1);
    func2_prim = @(t_i) ak2*wk2*cos(t_i*wk2+fi_k2);
    
    func1_bis = @(t_i) -1*ak1*wk1*wk1*sin(t_i*wk1+fi_k1);
    func2_bis = @(t_i) -1*ak2*wk2*wk2*sin(t_i*wk2+fi_k2);
    
    prog_pairs = [
        define_progressive_pair(6,'N',7,'M',0,rc60,rc70,func1,func1_prim,func1_bis);
        define_progressive_pair(8,'H',9,'G',0,rc80,rc90,func2,func2_prim,func2_bis);
        ];
    
    % nadawanie wartosci startowych

    % nadajemy wstepne polozenie czlonu(rc_i) oraz wstepna orientacje
    q0(1:3,1) = generate_start_values(rc10,rA0);
    q0(4:6,1) = generate_start_values(rc20,rO0);
    q0(7:9,1) = generate_start_values(rc30,rF0);
    q0(10:12,1) = generate_start_values(rc40,rA0);
    q0(13:15,1) = generate_start_values(rc50,rC0);
    q0(16:18,1) = generate_start_values(rc60,rN0);
    q0(19:21,1) = generate_start_values(rc70,rM0);
    q0(22:24,1) = generate_start_values(rc80,rH0);
    q0(25:27,1) = generate_start_values(rc90,rG0);
    q0(28:30,1) = generate_start_values(rc100,rE0);
    
    % kreacja czlonow
    
    body_0 = produce_body0(['O' 'N' 'H'],[rO0 rN0 rH0]);
    
    bodies = {
                % np. czlon 1 ( bodies(1) ) ma punkty A,D,P i wektor od ukladu
                % globalnego ukladu rc10
                produce_body(['A' 'D' 'P'],[rA0 rD0 rP0],rc10)
                produce_body(['O' 'P' 'F' 'G'],[rO0 rP0 rF0 rG0],rc20)
                produce_body(['F' 'E' 'C'],[rF0 rE0 rC0],rc30)
                produce_body(['A' 'K' 'B'],[rA0 rK0 rB0],rc40)
                produce_body(['C' 'B' ],[rC0 rB0],rc50)
                produce_body(['N'],[rN0],rc60)
                produce_body(['M'],[rM0],rc70)
                produce_body(['H'],[rH0],rc80)
                produce_body(['G'],[rG0],rc90)
                produce_body(['E' 'M' 'D'],[rE0 rM0 rD0],rc100) 
             };
    % dostep do tych pol jest nastepujacy , 
    % bodies{nr czlonu}{nr.punktu}.point lub local_vector
    % body = bodies{2}{3}.point
    % pobranie rozmiaru tego : s = size(bodies{2}) , s(2) to jest rozmiar
end

function result = define_progressive_pair(body_i,point_i,body_j,point_j,fi_i_j,r_i,r_j,driving_func,driving_func_prim,driving_func_bis)
    result.body_i = body_i;
    result.body_j = body_j;
    result.point_i = point_i;
    result.point_j = point_j;
    result.fi_i_j = fi_i_j;
    temp_vec = r_j - r_i;
    result.driving_versor = (temp_vec) / norm(temp_vec);
    result.perpendicular_versor = Rot(pi/2) * result.driving_versor;
    result.driving_versor = Rot(pi)*result.driving_versor;
    
    if nargin < 10
        result.driving_func = '';
        result.driving_func_prim = '';
        result.driving_func_bis = '';
    else
        result.driving_func = driving_func;
        result.driving_func_prim = driving_func_prim;
        result.driving_func_bis = driving_func_bis;
    end
end

function result = define_rotation_pair(body_i,body_j,point,driving_func,driving_func_prim,driving_func_bis)
    result.body_i = body_i;
    result.body_j = body_j;
    result.point = point;
    
    if nargin < 6
        result.driving_func = '';
        result.driving_func_prim = '';
        result.driving_func_bis = '';
    else
        result.driving_func = driving_func;
        result.driving_func_prim = driving_func_prim;
        result.driving_func_bis = driving_func_bis;
    end
end

% wazne jest zachowanie takiej samej kolejnosci punktow w obu argumentach
function result = produce_body(points_descriptors , global_vectors_to_points, rc_i) 
    for i = 1:strlength(points_descriptors)
        result{i} = produce_body_entity(points_descriptors(i) ,...
           get_vector_from_local_origin_to_point(global_vectors_to_points(:,i),rc_i));
%        result{i}.point 
%     result{i}.local_vec
    end
end

function result = produce_body0(points_descriptors , global_vectors_to_points) 
    for i = 1:strlength(points_descriptors)
        result{i} = produce_body_entity(points_descriptors(i) ,...
                                        global_vectors_to_points(:,i));
                                    result{i}.point
                                    result{i}.local_vec
    end
end

function result = produce_body_entity(point, local_vec)
    result.point = point;
    result.local_vec = local_vec;
end