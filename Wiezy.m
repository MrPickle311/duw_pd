function F=Wiezy(q,t,rot_pairs,prog_pairs,bodies,body0)
% Przypisanie elementom wektora q czytelnych nazw

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

% Rownania wiezow

% pary obrotowe

rot_size = size(rot_pairs);
rot_size = rot_size(1);

for n = 1:rot_size
    i = rot_pairs(n).body_i;
    j = rot_pairs(n).body_j;
    point = rot_pairs(n).point;
    
    if i == 0
        ri = 0;
        fi_i = fi0;
        s_i = get_local_vector_from_body_0(body0,point);
    else
        ri = q(3*i-2 : 3*i-1,1);
        fi_i = q(3*i,1);
        s_i = get_local_vector(bodies,i,point);
    end
    
    rj = q(3*j-2 : 3*j-1,1);
    fi_j = q(3*j,1);
    s_j = get_local_vector(bodies,j,point);

    F(2*n-1:2*n,1) = rotate_pair(ri,fi_i,s_i,rj,fi_j,s_j);
end



% para 6-7
fi_6_7 = 0;%-pi; % staly kat obrotu ukladu 6 wzgledem 7
vNM7 = [-1;4]/sqrt(17);%[0 -1]'; % wersor prostopad³y do osi ruchu wzglednego w ukladzie 6
dNM7 = Rot(pi/2)*vNM7;%Rot(pi/2)*vNM7; %[-1 0]'; % wersor ruchu wzglêdnego w ukladzie 6

% Para 8-9
fi_8_9 = 0; % staly kat obrotu ukladu 8 wzgledem 9
vHG8 = [-5;1]/sqrt(26);%[0 -1]'; % wersor prostopadly do osi ruchu wzglednego w ukladzie 8
dHG8 = Rot(pi/2)*vHG8;%Rot(pi/2)*vHG8;%[-1 0]'; % wersor ruchu wzglêdnego w ukladzie 8

% para postepowa 6-7 , NM
F(25:26,1) = progressive_pair(r6,fi6,bodies{6}{1}.local_vec,...
                              r7,fi7,bodies{7}{1}.local_vec,fi_6_7,vNM7);

% para postepowa 8-9 , HG
F(27:28,1) = progressive_pair(r8,fi8,bodies{8}{1}.local_vec,...
                              r9,fi9,bodies{9}{1}.local_vec,fi_8_9,vHG8);

% wiezy kierujace

% lk1 = sqrt(17);
% ak1 = 0.01;
% wk1 = 50;
% fi_k1 = 0.0;

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

temp = progressive_pair(r6,fi6,bodies{6}{1}.local_vec,...
                                                    r7,fi7,bodies{7}{1}.local_vec,fi_6_7,dNM7) - func1(t);

F(29,1) = temp(2);

temp = progressive_pair(r8,fi8,bodies{8}{1}.local_vec,...
                                                    r9,fi9,bodies{9}{1}.local_vec,fi_8_9,dHG8) - func2(t);

F(30,1) = temp(2);

% F(29,1) = relative_displacement_in_progressive_pair(r6,fi6,bodies{6}{1}.local_vec,...
%                                                     r7,fi7,bodies{7}{1}.local_vec,dNM7,func1,t);
%                                                 
% F(30,1) = relative_displacement_in_progressive_pair(r8,fi8,bodies{8}{1}.local_vec,...
%                                                     r9,fi9,bodies{9}{1}.local_vec,dHG8,func2,t);
% nb =4;
end
