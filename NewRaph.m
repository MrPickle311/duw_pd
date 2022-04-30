function q=NewRaph(q0,t)
% q=NewRaph(q0,t)
%   Rozwi¹zanie uk³adu równañ metod¹ Newtona-Raphsona.
% Wejœcie:
%   q0 - przybli¿enie startowe,
%   t - chwila, dla której poszukiwane jest rozwi¹zanie.
% Wyjœcie:
%   q - uzyskane rozwi¹zanie.
%
% Uk³ad równañ musi byæ wpisany w pliku Wiezy.m.
% Macierz Jacobiego uk³adu równañ musi byæ wpisana w pliku Jakobian.m.

q=q0;
F=Wiezy(q,t);
iter=1; % Licznik iteracji
while ( (norm(F)>1e-10) & (iter < 25) )
    F=Wiezy(q,t);
    Fq=Jakobian(q);
    q=q-Fq\F;  % Fq\F jest równowa¿ne inv(Fq)*F, ale mniej kosztowne numerycznie
    iter=iter+1;
end
if iter >= 25
    disp('Blad: Po 25 iteracjach Newtona-Raphsona nie uzyskano zbie¿noœci ');
    q=q0;
end

