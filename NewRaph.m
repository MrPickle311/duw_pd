function q=NewRaph(q0,t,rot_pairs,bodies,body0)
q=q0;
F=Wiezy(q,t,rot_pairs,bodies,body0);
iter=1; % Licznik iteracji
while ( (norm(F)>1e-10) && (iter < 25) )
    F=Wiezy(q,t,rot_pairs,bodies,body0);
    Fq=Jakobian(q,rot_pairs,bodies,body0);
    q=q-Fq\F;  % Fq\F jest równowa¿ne inv(Fq)*F, ale mniej kosztowne numerycznie
    iter=iter+1;
end
if iter >= 25
    disp('Blad: Po 25 iteracjach Newtona-Raphsona nie uzyskano zbie¿noœci ');
    q=q0;
end

