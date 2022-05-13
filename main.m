function [timespan,q_i,dq_i,d2q_i] = main()

    size = 30;
    lroz=0; % Licznik rozwiązań (służy do numerowania kolumn w tablicach z wynikami)
    
    t0=0;
    dt=0.05; % Odstęp pomiędzy kolejnymi chwilami
    tk = 1;
    
    % wczytaj dane dla ukladu
    [rot_pairs,prog_pairs,body0,bodies,q] = moje_dane();
    
    q_prim=zeros(size,1); 
    q_bis=zeros(size,1);

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
        timespan(1,lroz)=t; 
        q_i(:,lroz)=q;
        dq_i(:,lroz)=q_prim;
        d2q_i(:,lroz)=q_bis;
    end
    
    % testy jednostkowe
    
    disp('Testowanie...')
    
    test_data = load('matlab.mat');
    
    q_test = test_data.q_i;
    dq_test = test_data.dq_i;
    d2q_test = test_data.d2q_i;
    
    is_q_eq = isequal(q_test,q_i);
    is_dq_eq = isequal(dq_test,dq_i);
    is_d2q_eq = isequal(d2q_test,d2q_i);
    
    if ~is_q_eq
        error('Polozenia sie nie zgadzaja z danymi testowymi')
    end
    
    if ~is_dq_eq
        error('Predkosci sie nie zgadzaja z danymi testowymi')
    end
    
    if ~is_d2q_eq
        error('Przyspieszenia sie nie zgadzaja z danymi testowymi')
    end
    
    disp('Zakonczono testowanie')
    
end