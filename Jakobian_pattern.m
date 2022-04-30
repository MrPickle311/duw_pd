function Fq=Jakobian(q)
% Fq=Jakobian(q)
%   Procedura wspó³pracuj¹ca z NewRaph.
%   S³u¿y do obliczania macierzy Jacobiego równañ wiêzów.
% Wejœcie:
%   q - wspó³rzêdne absolutne uk³adu wielocz³onowego.
% Wyjœcie:
%   Fq - obliczona macierz Jacobiego.
%
%*************************************************************
%   Program stanowi za³¹cznik do ksi¹¿ki:
%   Fr¹czek J., Wojtyra M.: Kinematyka uk³adów wielocz³onowych.
%                           Metody obliczeniowe. WNT 2007.
%   Wersja 1.0
%*************************************************************


% Wymiary mechanizmu
AB=0.1;     LI=AB;
AD=0.4;     HL=AD;
BC=0.225;   GI=BC;
CD=0.3;     GH=CD;
CE=0.25;    FG=CE;
CF=0.2;     EG=CF;
%a=0.01;     b=0.2;
CM=0.05;    GN=0.06;
EM=0.24;    FN=0.24;

kECM=acos((EM^2-CM^2-CE^2)/(-2*CM*CE));
kFGN=acos((FN^2-GN^2-FG^2)/(-2*GN*FG));

sA0=[0;0];                          sA1=[0;0];      %Para obrotowa 0-1, pkt A +
sB1=[AB; 0];                        sB2=[0;0];      %Para obrotowa 1-2, pkt B +
sC2=[BC;0];                         sC3=[CD;0];     %Para obrotowa 2-3, pkt C +
sD0=[AD;0];                         sD3=[0;0];      %Para obrotowa 0-3, pkt D+
sE3=[CD+CE;0];                      sE4=[0;0];      %Para obrotowa 3-4, pkt E
sF2=[BC+CF;0];                      sF5=[0;0];      %Para obrotowa 2-5, pkt F
sG4=[EG;0];                         sG5=[FG;0];     %Para obrotowa 4-5, pkt G
sH5=[FG+GH;0];                      sH6=[0;0];      %Para obrotowa 5-6, pkt H
sI4=[EG+GI;0];                      sI7=[0;0];      %Para obrotowa 4-7, pkt I
sL7=[LI;0];                         sL6=[HL;0];     %Para obrotowa 7-6, pkt L
sM3=[CD+CM*cos(kECM);-CM*sin(kECM)]; sM8=[0;0];      %Para obrotowa 3-8, pkt M-
sN5=[FG-GN*cos(kFGN); GN*sin(kFGN)]; sN9=[0;0];      %Para obrotowa 5-9, pkt N
fio89=pi; v9=[0;-1]; 

%Wiêzy kieruj¹ce:
u9=[-1;0];

r0=[0;0]; fi0=0;

% Przypisanie elementom wektora q czytelnych nazw
r1=q(1:2);   fi1=q(3);    r2=q(4:5);   fi2=q(6);    r3=q(7:8);     fi3=q(9);
r4=q(10:11); fi4=q(12);   r5=q(13:14); fi5=q(15);   r6=q(16:17);   fi6=q(18);
r7=q(19:20); fi7=q(21);   r8=q(22:23); fi8=q(24);   r9=q(25:26);   fi9=q(27);

% Macierz Jaocobiego - pocz¹tkowo zerowa
Fq=zeros(27,27);

% Macierz Jacobiego - wype³nianie niezerowych elementów
%Pary obrotowe
% elegancka mapka
% A01 - B12 - C23 - D03 - E34 - F25 - G45 - H56 - I47 - L67 - M38 - N59
[tmp(1:2,1:6)]                  =para_obr_j(fi0,sA0,fi1,sA1);   
Fq(1:2,1:3)                     =tmp(1:2,4:6);

Fq(3:4,1:6)                     =para_obr_j(fi1,sB1,fi2,sB2);
Fq(5:6,4:9)                     =para_obr_j(fi2,sC2,fi3,sC3);

tmp(1:2,1:6)                    =para_obr_j(fi0,sD0,fi3,sD3); 
Fq(7:8,7:9)                     =tmp(1:2,4:6);

Fq(9:10,7:12)                   =para_obr_j(fi3,sE3,fi4,sE4);
Fq(11:12,[4:6 13:15])           =para_obr_j(fi2,sF2,fi5,sF5);
Fq(13:14,10:15)                 =para_obr_j(fi4,sG4,fi5,sG5);
Fq(15:16,13:18)                 =para_obr_j(fi5,sH5,fi6,sH6);

Fq(17:18,[10:12 19:21])         =para_obr_j(fi4,sI4,fi7,sI7);         
Fq(19:20,16:21)                 =para_obr_j(fi6,sL6,fi7,sL7);
Fq(21:22,[7:9 22:24])           =para_obr_j(fi3,sM3,fi8,sM8);
Fq(23:24,[13:15 25:27])         =para_obr_j(fi5,sN5,fi9,sN9);
%Para postêpowa
Fq(25:26, 22:27) = para_post_j(r8,fi8,sM8,r9,fi9,v9);
%wiêzy kieruj¹ce
tmp(1:2,1:6) = para_post_j(r8,fi8,sM8,r9,fi9,u9);
Fq(27,22:27) = tmp(2,1:6);

end