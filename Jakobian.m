function result=Jakobian(q,rot_pairs,bodies,body0)

% Macierz Jaocobiego - pocz¹tkowo zerowa
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

% Obliczenie macierzy kosinusów kierunkowych
%Rot1=Rot(fi1);  Rot2=Rot(fi2);  Rot3=Rot(fi3);

% Macierz Jaocobiego - wypelnianie niezerowych elementów

% pary obrotowe

rot_size = size(rot_pairs);

for n = 1:2:rot_size
    i = rot_pairs(n).body_i;
    j = rot_pairs(n).body_j;
    point = rot_pairs(n).point;
    
    if i == 0
        fi_i = fi0;
        s_i = get_local_vector_from_body_0(body0,point);
    else
        fi_i = q(3*i);
        s_i = get_local_vector(bodies,i,point);
    end
    
    fi_j = q(3*j);
    
    s_j = get_local_vector(bodies,j,point);

    result = insert_rotation_pair_into_jacobi(result,i,j,n,...
                                        fi_i,s_i,...
                                        fi_j,s_j);
end

% DLACZEGO tu jest 1 na koncu
%result(7,9)=1;
%result(8,7:8)=-v0';
%result(8,9)=-v0'*Om*Rot3*sA3;
%result(9,3)=1; 
