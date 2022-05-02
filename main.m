function [T,Q] = main()

    size = 30;
    %q=zeros(size,1);
    lroz=0; % Licznik rozwiązań (służy do numerowania kolumn w tablicach z wynikami)
    
    t0=0;
    dt=0.05; % Odstęp pomiędzy kolejnymi chwilami
    tk = 1;
    
    % wczytaj dane dla ukladu
    [rot_pairs,body0,bodies,q0] = moje_dane();
    
    
    %dq=zeros(size,1); 
    %d2q=zeros(size,1);
    
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

   %FI = @(t_i,q_i) Wiezy(q_i,t_i,rot_pairs,bodies,body0);
%    Q = fsolve(FI,q);
%       T= 4;
    %[T,Q] = ode45(FI,[t0 tk],q);
%     Q = 3;
    for t=t0:dt:tk
        %q0=q+dq*dt+0.5*d2q*dt^2;
        q=NewRaph(q,t,rot_pairs,bodies,body0);
        lroz=lroz+1;
        T(1,lroz)=t; 
        Q(:,lroz)=q;
    end
end