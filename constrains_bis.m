function result = constrains_bis(q,q_prim,t,rot_pairs,prog_pairs,bodies,body0)

% kinematyczne
% obrotowe : wzór 2.42
% postępowe : pierwszy wiersz zerowy, drugi :wzór 2.59
% driving
% obrotowe: pierwszy wiersz: 
% postępowe : wzór 2.59 + druga_pochodna_func(t)

r0=[0;0];       fi0 = 0;  
r6=q(16:17);    fi6=q(18);
r7=q(19:20);    fi7=q(21);   
r8=q(22:23);    fi8=q(24);   
r9=q(25:26);    fi9=q(27);

r_prim_0=[0;0];            fi_prim_0 = 0;   
r_prim_6=q_prim(16:17);    fi_prim_6=q_prim(18);
r_prim_7=q_prim(19:20);    fi_prim_7=q_prim(21);   
r_prim_8=q_prim(22:23);    fi_prim_8=q_prim(24);   
r_prim_9=q_prim(25:26);    fi_prim_9=q_prim(27);

rot_size = size(rot_pairs);

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

fi_6_7 = 0;%-pi; % staly kat obrotu ukladu 6 wzgledem 7
vNM7 = [-1;4]/sqrt(17);%[0 -1]'; % wersor prostopad³y do osi ruchu wzglednego w ukladzie 6
dNM7 = Rot(pi/2)*vNM7;%[-4;-1]/sqrt(17);%; %[-1 0]'; % wersor ruchu wzglêdnego w ukladzie 6

% Para 8-9
fi_8_9 = 0; % staly kat obrotu ukladu 8 wzgledem 9
vHG8 = [-5;1]/sqrt(26);%[0 -1]'; % wersor prostopadly do osi ruchu wzglednego w ukladzie 8
dHG8 = Rot(pi/2)*vHG8;%[-1;5]/sqrt(26);%%[-1 0]'; % wersor ruchu wzglêdnego w ukladzie 8

% para postepowa 6-7 , NM
result(25:26,1) = progressive_acceleration(r6,r_prim_6,fi6,fi_prim_6,bodies{6}{1}.local_vec,...
                                      r7,r_prim_7,fi7,fi_prim_7,vNM7);

% para postepowa 8-9 , HG
result(27:28,1) = progressive_acceleration(r8,r_prim_8,fi8,fi_prim_8,bodies{8}{1}.local_vec,...
                                      r9,r_prim_9,fi9,fi_prim_9,vHG8);
lk1 = sqrt(17)/5;
ak1 = 0.1;
wk1 = 0.1;
fi_k1 = 0.0;

lk2 = sqrt(26)/5;
ak2 = 0.1;
wk2 = 0.1;
fi_k2 = 0.0;

func1_bis = @(t_i) -1*ak1*wk1*wk1*sin(t*wk1+fi_k1);
func2_bis = @(t_i) -1*ak2*wk2*wk2*sin(t*wk2+fi_k2);

temp = progressive_acceleration(r6,r_prim_6,fi6,fi_prim_6,bodies{6}{1}.local_vec,...
                                      r7,r_prim_7,fi7,fi_prim_7,dNM7) + func1_bis(t);

result(29,1) = temp(2);

temp = progressive_acceleration(r8,r_prim_8,fi8,fi_prim_8,bodies{8}{1}.local_vec,...
                                      r9,r_prim_9,fi9,fi_prim_9,dHG8) + func2_bis(t);

result(30,1) = temp(2);
   
con = 5;

end

