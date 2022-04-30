clear
clc
[rot_pairs,body_0,bodies] = moje_dane();
q0 = zeros(30,1);
result=Jakobian(q0,rot_pairs,bodies,body_0);