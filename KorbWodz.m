function [T,Q]=KorbWodz()
% [T,Q,DQ,D2Q]=KorbWodz
% Rozwiązanie zadań kinematyki o położeniu, prędkości i przyspieszeniu 
%   dla mechanizmu korbowo-wodzikowego.
% Wyjście:
%   T   - tablica do zapisu kolejnych chwil.
%   Q   - tablica do zapisu rozwiązań zadania o położeniu w kolejnych chwilach.
%   DQ  - tablica do zapisu rozwiązań zadania o prędkości w kolejnych chwilach.
%   D2Q - tablica do zapisu rozwiązań zad. o przyspieszeniu w kolejnych chwilach.

size = 30;

% Przybliżenie startowe (gdy brak rozwiązania z poprzedniej chwili)
q=zeros(size,1);
dq=zeros(size,1); 
d2q=zeros(size,1);
lroz=0; % Licznik rozwiązań (służy do numerowania kolumn w tablicach z wynikami)
dt=0.05; % Odstęp pomiędzy kolejnymi chwilami

moje_dane;

% Rozwiązywanie zadań kinematyki w kolejnych chwilach t
for t=0:dt:1.5
    % Zadanie o położeniu. 
    % Przybliżeniem początkowym jest rozwiązanie z poprzedniej chwili, 
    % powiększone o składniki wynikające z obliczonej prędkości i przyspieszenia.
    q0=q+dq*dt+0.5*d2q*dt^2;
    q=NewRaph(q0,t); 
    
    %dq=Predkosc(q,t);  % Zadanie o prędkości

    %d2q=Przyspieszenie(dq,q,t);  % Zadanie o przyspieszeniu

    % Zapis do tablic gromadzących wyniki 
    lroz=lroz+1;
    T(1,lroz)=t; 
    Q(:,lroz)=q;
    %DQ(:,lroz)=dq;
    %D2Q(:,lroz)=d2q;
end
