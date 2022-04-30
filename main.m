function [T,Q] = main()
    clear
    clc
    size = 30;
    q=zeros(size,1);
    lroz=0; % Licznik rozwiązań (służy do numerowania kolumn w tablicach z wynikami)
    dt=0.05; % Odstęp pomiędzy kolejnymi chwilami
    
    
    % wczytaj dane dla ukladu
    [rot_pairs,body0,bodies] = moje_dane();
    
    q=zeros(size,1);
    q(1) = 5;
    dq=zeros(size,1); 
    d2q=zeros(size,1);
    t = 1;
%     result=Jakobian(q0,rot_pairs,bodies,body_0);
%     F=Wiezy(q0,t,rot_pairs,bodies,body_0);
    
    for t=0:dt:4.0
        q0=q+dq*dt+0.5*d2q*dt^2;
        q=NewRaph(q0,t,rot_pairs,bodies,body0);
        lroz=lroz+1;
        T(1,lroz)=t; 
        Q(:,lroz)=q;
    end
end