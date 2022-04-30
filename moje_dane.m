function [rot_pairs,body_0,bodies,fi_6_7,dNM7,vNM7,fi_8_9,dHG8,vHG8] = moje_dane()
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

    % definicja par obrotowych
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

    %nazewnictwo -> s<numer_czlonu><Punkt>_<numer_ukladu_odniesienia>

    % czlon 0
    s0O_0 = rO0;
    s0N_0 = rN0;
    s0H_0 = rH0;

    % obliczanie wektorow s w ukladach lokalnych ( stale wektory w lokalnych
    % ukladach odniesienia)

    s1A_1 = get_vector_from_local_origin_to_point(rA0,rc10);
    s1D_1 = get_vector_from_local_origin_to_point(rD0,rc10);
    s1P_1 = get_vector_from_local_origin_to_point(rP0,rc10);
    % czlon 2
    s2O_2 = get_vector_from_local_origin_to_point(rO0,rc20);
    s2P_2 = get_vector_from_local_origin_to_point(rP0,rc20);
    s2F_2 = get_vector_from_local_origin_to_point(rF0,rc20);
    s2G_2 = get_vector_from_local_origin_to_point(rG0,rc20);
    % czlon 3
    s3F_3 = get_vector_from_local_origin_to_point(rF0,rc30);
    s3E_3 = get_vector_from_local_origin_to_point(rE0,rc30);
    s3C_3 = get_vector_from_local_origin_to_point(rC0,rc30);
    % czlon 4
    s4A_4 = get_vector_from_local_origin_to_point(rA0,rc40);
    s4K_4 = get_vector_from_local_origin_to_point(rK0,rc40);
    s4B_4 = get_vector_from_local_origin_to_point(rB0,rc40);
    % czlon 5
    s5C_5 = get_vector_from_local_origin_to_point(rC0,rc50);
    s5B_5 = get_vector_from_local_origin_to_point(rB0,rc50);
    % czlon 6
    s6N_6 = get_vector_from_local_origin_to_point(rN0,rc60);
    % czlon 7
    s7M_7 = get_vector_from_local_origin_to_point(rM0,rc70);
    % czlon 8
    s8H_8 = get_vector_from_local_origin_to_point(rH0,rc80);
    % czlon 9
    s9G_9 = get_vector_from_local_origin_to_point(rG0,rc90);
    % czlon 10
    s10E_10 = get_vector_from_local_origin_to_point(rE0,rc100);
    s10M_10 = get_vector_from_local_origin_to_point(rM0,rc100);
    s10D_10 = get_vector_from_local_origin_to_point(rD0,rc100);

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

    % para 6-7
    fi_6_7 = -pi; % staly kat obrotu ukladu 6 wzgledem 7
    dNM7 = [1 0]'; % wersor ruchu wzglêdnego w ukladzie 6
    vNM7 = [0 1]'; % wersor prostopad³y do osi ruchu wzglednego w ukladzie 6
    % Para 8-9
    fi_8_9 = -pi; % staly kat obrotu ukladu 8 wzgledem 9
    dHG8 = [1 0]'; % wersor ruchu wzglêdnego w ukladzie 8
    vHG8 = [0 1]'; % wersor prostopadly do osi ruchu wzglednego w ukladzie 8
end

function result = define_rotation_pair(body_i,body_j,point)
    result.body_i = body_i;
    result.body_j = body_j;
    result.point = point;
end

% wazne jest zachowanie takiej samej kolejnosci punktow w obu argumentach
function result = produce_body(points_descriptors , global_vectors_to_points, rc_i) 
    for i = 1:strlength(points_descriptors)
        result{i} = produce_body_entity(points_descriptors(i) ,...
        get_vector_from_local_origin_to_point(global_vectors_to_points(i),rc_i));
    end
end

function result = produce_body0(points_descriptors , global_vectors_to_points) 
    for i = 1:strlength(points_descriptors)
        result{i} = produce_body_entity(points_descriptors(i) ,...
                                        global_vectors_to_points(:,i));
    end
end

function result = produce_body_entity(point, local_vec)
    result.point = point;
    result.local_vec = local_vec;
end