function result=Jakobian(q,rot_pairs,prog_pairs,bodies,body0)

% Macierz Jaocobiego - początkowo zerowa
result=zeros(30,30);

% definicja par obrotowych

r0=[0;0];       fi0 = 0;
r1=q(1:2);      fi1=q(3);    
r2=q(4:5);      fi2=q(6);    
r3=q(7:8);      fi3=q(9);
r4=q(10:11);    fi4=q(12);   
r5=q(13:14);    fi5=q(15);   
r6=q(16:17);    fi6=q(18);
r7=q(19:20);    fi7=q(21);   
r8=q(22:23);    fi8=q(24);   
r9=q(25:26);    fi9=q(27);
r10=q(28:29);   fi10=q(30);

% Macierz Jaocobiego - wypelnianie niezerowych elementów

% pary obrotowe

rot_size = size(rot_pairs);
rot_size = rot_size(1);
prog_size = size(prog_pairs);
prog_size = prog_size(1);

for n = 1:rot_size
    i = rot_pairs(n).body_i;
    j = rot_pairs(n).body_j;
    point = rot_pairs(n).point;
    
    [ri,fi_i,s_i,rj,fi_j,s_j] = get_current_data(q,bodies,body0,i,j,point,point);

    result = insert_rotation_pair_into_jacobi(result,i,j,n,...
                                        fi_i,s_i,...
                                        fi_j,s_j);
end

% pary postepowe

for n = 1:prog_size
    i = prog_pairs(n).body_i;
    j = prog_pairs(n).body_j;
    point_i = prog_pairs(n).point_i;
    point_j = prog_pairs(n).point_j;
    perp = prog_pairs(n).perpendicular_versor;
    
    [ri,fi_i,s_i,rj,fi_j,s_j] = get_current_data(q,bodies,body0,i,j,point_i,point_j);

    current_row = 2*rot_size+2*n - 1;
    
    result = insert_progressive_pair(result,i,j,current_row,...
                                        ri,fi_i,s_i,...
                                        rj,fi_j,perp);
end

% para 6-7
fi_6_7 = 0;%-pi; % staly kat obrotu ukladu 6 wzgledem 7
vNM7 = [-1;4]/sqrt(17);%[0 -1]'; % wersor prostopad³y do osi ruchu wzglednego w ukladzie 6
dNM7 = Rot(pi/2)*vNM7;%[-4;-1]/sqrt(17);%; %[-1 0]'; % wersor ruchu wzglêdnego w ukladzie 6

% Para 8-9
fi_8_9 = 0; % staly kat obrotu ukladu 8 wzgledem 9
vHG8 = [-5;1]/sqrt(26);%[0 -1]'; % wersor prostopadly do osi ruchu wzglednego w ukladzie 8
dHG8 = Rot(pi/2)*vHG8;%[-1;5]/sqrt(26);%%[-1 0]'; % wersor ruchu wzglêdnego w ukladzie 8

 % wiezy kierujace
tmp(1:2,1:6) = jacobi_element_for_progressive_pair(r6,fi6,get_local_vector_from_body(bodies{6},'N'), ...
                                                   r7,fi7,dNM7);
result(29,16:21) = tmp(2,1:6);

tmp(1:2,1:6) = jacobi_element_for_progressive_pair(r8,fi8,get_local_vector_from_body(bodies{8},'H'), ...
                                                   r9,fi9,dHG8); 
result(30,22:27) = tmp(2,1:6);

epsilon = 1e-6;
if (cond(result) > (1/epsilon()))
    error('Macierz Jacobiego osobliwa');
end

m = 7;
end
