function [T,Q,DQ,D2Q] = main()

    size = 30;
    %q=zeros(size,1);
    lroz=0; % Licznik rozwiązań (służy do numerowania kolumn w tablicach z wynikami)
    
    t0=0;
    dt=0.05; % Odstęp pomiędzy kolejnymi chwilami
    tk = 1;
    
    % wczytaj dane dla ukladu
    [rot_pairs,prog_pairs,body0,bodies,q0] = moje_dane();
    
    q_prim=zeros(size,1); 
    q_bis=zeros(size,1);
    
    q = [0.7; -0.2; 0;
        0; 0.2; 0;
        0.2; 0.3; 0;
        1.55; -0.35; 0;
        0.9; 0.2; 0;
        0.2; -0.35; 0;
        0.6; -0.25; 0;
        0.15; -0.45; 0;
        0.25; 0.05; 0;
        0.7; 0; 0];

    for t=t0:dt:tk
        q0=q+q_prim*dt+0.5*q_bis*dt^2;
        
        q=NewRaph(q0,t,rot_pairs,prog_pairs,bodies,body0);
        % predkosci
        q_prim=  Jakobian(q0,rot_pairs,prog_pairs,bodies,body0) \...
            constrain_first_dot(q0,t,rot_pairs,prog_pairs,bodies,body0);
        % przyspieszenia
        q_bis=Jakobian(q0,rot_pairs,prog_pairs,bodies,body0)\...
            constrains_bis(q0,q_prim,t,rot_pairs,prog_pairs,bodies,body0);
        
        lroz=lroz+1;
        T(1,lroz)=t; 
        Q(:,lroz)=q;
        DQ(:,lroz)=q_prim;
        D2Q(:,lroz)=q_bis;
    end
    %plot(T(:,1),Q(:,1))
end