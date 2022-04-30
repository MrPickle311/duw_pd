function F=Wiezy(q,t)
% F=Wiezy(q,t)
%   Procedura wspó³pracuj¹ca z NewRaph.
%   S³u¿y do obliczania wartoœci funkcji opisuj¹cych wiêzy.
% Wejœcie:
%   q - wspó³rzêdne absolutne uk³adu wielocz³onowego,
%   t - aktualna chwila.
% Wyjœcie:
%   F - wartoœci funkcji.



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

%1 - definiujemy 

sA0=[0;0];                          sA1=[0;0];      %Para obrotowa 0-1, pkt A
sB1=[AB; 0];                        sB2=[0;0];      %Para obrotowa 1-2, pkt B
sC2=[BC;0];                         sC3=[CD;0];     %Para obrotowa 2-5, pkt C
sD0=[AD;0];                         sD3=[0;0];      %Para obrotowa 0-5, pkt D
sE3=[CD+CE;0];                      sE4=[0;0];      %Para obrotowa 5-6, pkt E
sF2=[BC+CF;0];                      sF5=[0;0];      %Para obrotowa 2-7, pkt F
sG4=[EG;0];                         sG5=[FG;0];     %Para obrotowa 6-7, pkt G
sH5=[FG+GH;0];                      sH6=[0;0];      %Para obrotowa 7-9, pkt H
sI4=[EG+GI;0];                      sI7=[0;0];      %Para obrotowa 6-8, pkt I
sL7=[LI;0];                         sL6=[HL;0];     %Para obrotowa 8-9, pkt L
sM3=[CD+CM*cos(kECM);-CM*sin(kECM)]; sM8=[0;0];      %Para obrotowa 1-3, pkt M
sN5=[FG-GN*cos(kFGN); GN*sin(kFGN)]; sN9=[0;0];      %Para obrotowa 2-4, pkt N


fio89=pi; 
v9=[0;-1]; % wektor prostopadly do osi x lokalnego ukladu            

%Wiêzy kieruj¹ce:
u9=[-1;0]; % 

r0=[0;0]; fi0=0;

% Przypisanie elementom wektora q czytelnych nazw
r1=q(1:2);   fi1=q(3);    r2=q(4:5);   fi2=q(6);    r3=q(7:8);     fi3=q(9);
r4=q(10:11); fi4=q(12);   r5=q(13:14); fi5=q(15);   r6=q(16:17);   fi6=q(18);
r7=q(19:20); fi7=q(21);   r8=q(22:23); fi8=q(24);   r9=q(25:26);   fi9=q(27);

% Równania wiêzów (najpierw pary obrotowe w kol. alfabet., nast. si³ownik
% A01 - B12 - C23 - D03 - E34 - F25 - G45 - H56 - I47 - L67 - M38 - N59
F(1:2,1)=   para_obr_wiezy(r0,fi0,sA0,r1,fi1,sA1);
F(3:4,1)=   para_obr_wiezy(r1,fi1,sB1,r2,fi2,sB2);
F(5:6,1)=   para_obr_wiezy(r2,fi2,sC2,r3,fi3,sC3);
F(7:8,1)=   para_obr_wiezy(r0,fi0,sD0,r3,fi3,sD3);
F(9:10,1)=  para_obr_wiezy(r3,fi3,sE3,r4,fi4,sE4);
F(11:12,1)= para_obr_wiezy(r2,fi2,sF2,r5,fi5,sF5);
F(13:14,1)= para_obr_wiezy(r4,fi4,sG4,r5,fi5,sG5);
F(15:16,1)= para_obr_wiezy(r5,fi5,sH5,r6,fi6,sH6);
F(17:18,1)= para_obr_wiezy(r4,fi4,sI4,r7,fi7,sI7);
F(19:20,1)= para_obr_wiezy(r6,fi6,sL6,r7,fi7,sL7);
F(21:22,1)= para_obr_wiezy(r3,fi3,sM3,r8,fi8,sM8);
F(23:24,1)= para_obr_wiezy(r5,fi5,sN5,r9,fi9,sN9);
%para postêpowa 8-9
F(25:26,1)=para_post_wiezy(r8,fi8,sM8,r9,fi9,sN9,v9,fio89);
%wiêzy kieruj¹ce
tmp=para_post_wiezy(r8,fi8,sM8,r9,fi9,sN9,u9,fio89);
F(27,1)=tmp(2,1)-funkcja(t);

dfd=4

end