function q=NewRaph(q0,t,rot_pairs,bodies,body0)
    max_iter = 25;
    eps = 1e-4;
    q=q0;
    F=Wiezy(q,t,rot_pairs,bodies,body0);

%     FI = @(q_i) Wiezy(q_i,t,rot_pairs,bodies,body0);
%     q = fsolve(FI,q0)

    iter=1; % Licznik iteracji
    
    while ( (norm(F)>eps) && (iter < max_iter) )
        F=Wiezy(q,t,rot_pairs,bodies,body0);
        Fq=Jakobian(q,rot_pairs,bodies,body0);
        q=q-Fq\F;
        iter=iter+1;
    end
    if iter >= max_iter
        X = sprintf('Blad: Po %i iteracjach Newtona-Raphsona nie uzyskano zbie¿noœci dla czasu t = %f'...
            ,max_iter,t);
        disp(X)
        q=q0;
    end
end

