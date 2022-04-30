function result=jacobi_element_for_progressive_pair(ri,fi_i,sa_i,rj,fi_j,v_j)
    OM=[0 -1;
        1  0];

    %katowe wiezy kinematyczne zroniczkowane po ri,fi_i,rj,fi_j
    result(1,1:6)=[0 0 1 0 0 -1];        %wzor 2.43-2.46
    %postepowe wiezy kinematyczne zrozniczkowane po ri
    result(2,1:2)=-(Rot(fi_j)*v_j)';       %wzor 2.49
    %postepowe wiezy kinematyczne zrozniczkowane po fi_i
    result(2,3)=-(Rot(fi_j)*v_j)'*OM*Rot(fi_i)*sa_i;   %wzor 2.50
    %postepowe wiezy kinematyczne zrozniczkowane po rj
    result(2,4:5)=(Rot(fi_j)*v_j)';        %wzor 2.51
    %postepowe wiezy kinematyczne zrozniczkowane po fi_j
    result(2,6)=-(Rot(fi_j)*v_j)'*OM*(rj-ri-Rot(fi_i)*sa_i); %wz. 2.52
end