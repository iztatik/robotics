%Diplomova praca
%Simulacia ramena SEF-ROBOTER
%Frantisek Durovsky
%2012

%====================================================
clear 
clc

%Parametre clankov - tabulka Denavit Hartenbergovych parametrov
%Struktura ramena RRRRRR - premenne tu theta1-theta6

%clanok1
alfa1 = pi/2;
a1 = 0.1;
d1 = 0.3;

%clanok2
a2 = 0.8;

%clanok3
alfa3 = pi/2;
a3 = 0.0;

%clanok4 
alfa4 = -pi/2;
d4 = 0.743;

%clanok5 
alfa5 = pi/2;
a5 = 0.0;

%clanok6
 d6 = 0.2;
 alfa6 = pi/2;

%Premenne vo vychodzom bode[-2 2 4 4 6 6]
theta1 = 0;
theta2 = -2;
theta3 = -3;
theta4 = 0;
theta5 = 0;
theta6 = 0;
%==========================================================================
%Transformacne matice medzi jednotlivymi klbmi

%absolutne vyjadrenie
T10 = trotz(theta1)*transl(0,0,d1)*transl(a1,0,0)*round(trotx(alfa1));
T21 = trotz(theta2)*transl(a2,0,0);
T32 = trotz(theta3)*round(trotx(alfa3));
T43 = trotz(theta4)*transl(0,0,d4)*round(trotx(alfa4));
T54 = trotz(theta5)*transl(a5,0,0)*round(trotx(alfa5));
T65 = trotz(theta6)*round(trotx(alfa6));

T = T10*T21*T32*T43*T54*T65

%symbolicke vyjadrenie
 syms t1 t2 t3 t4 t5 t6 s_d1 s_a1 s_a2 s_d4 s_a5 s_d6
 s_T10 = trotz(t1)*transl(0,0,s_d1)*transl(s_a1,0,0)*round(trotx(alfa1))
 s_T21 = trotz(t2)*transl(s_a2,0,0)
 s_T32 = trotz(t3)*round(trotx(alfa3))
 s_T43 = trotz(t4)*transl(0,0,s_d4)*round(trotx(alfa4))
 s_T54 = trotz(t5)*transl(s_a5,0,0)*round(trotx(alfa5))
 s_T65 = trotz(t6)*transl(0,0,s_d6)*round(trotx(alfa6))
  
 s_T = s_T10*s_T21*s_T32*s_T43*s_T54*s_T65

%==========================================================================
%Model ramena

%           theta   d       a          alfa
L(1) = Link([ 0     d1     a1        -pi/2     0], 'standard');
L(2) = Link([ 0 	0      a2	       0       0], 'standard');
L(3) = Link([ 0     0      0          pi/2     0], 'standard');
L(4) = Link([ 0     d4	   0         -pi/2     0], 'standard');
L(5) = Link([ 0     0      a5         pi/2     0], 'standard');
L(6) = Link([ 0     d6	   0          pi/2     0], 'standard');

q_lim = [-0.875*pi        0.875*pi-pi/4;
         -0.54*pi-pi/4    0.54*pi-pi/4;
         -0.69*pi+5*pi/8  0.69*pi+5*pi/8;
         -pi              pi;
         -1.1*pi          1.1*pi;
         -pi              pi];

q1 = [0.0 -2  3   0   0 0.5];
hold off
KEM = SerialLink(L,'name','KEM-ROBOT','manufacturer','SEF-ROBOTER','qlim',q_lim);
KEM.plot(q1);
%==========================================================================
%Body pohybu

P1 = [1 0 0 .8;
      0 1 0 .4;
      0 0 1 1.5;
      0 0 0 1];

P2 = [1 0 0  -1;
      0 1 0  1;
      0 0 1   0.5;
      0 0 0  1];
  
P3 = [1 0 0 .5;
      0 1 0 .5;
      0 0 1 .5;
      0 0 0 1];
  

P4 = [1 0 0 -1;
      0 1 0  1;
      0 0 1  1;
      0 0 0  1];
      
q_p1 = KEM.ikine(P1)
q_p2 = KEM.ikine(P2)
q_p3 = KEM.ikine(P3)
q_p4 = KEM.ikine(P4)

%=========================================================================
%Trajektoria 1

pohyb1 = KEM.jtraj(P1,P2,50);
q_f1 = KEM.fkine(pohyb1)

x_traj = zeros(1,50);
y_traj = zeros(1,50);
z_traj = zeros(1,50);

for i=1:50
x_traj(1,i) = q_f1(1,4,i);
y_traj(1,i) = q_f1(2,4,i);
z_traj(1,i) = q_f1(3,4,i);
end

hold on
[x,y,z] = sphere(16);
scatter3(x_traj,y_traj,z_traj,'.');

%=========================================================================
%Trajektoria 2

pohyb2 = KEM.jtraj(P2,P3,50);
q_f2 = KEM.fkine(pohyb2)

x_traj = zeros(1,50);
y_traj = zeros(1,50);
z_traj = zeros(1,50);


for i=1:50
x_traj(1,i) = q_f2(1,4,i);
y_traj(1,i) = q_f2(2,4,i);
z_traj(1,i) = q_f2(3,4,i);
end

hold on
[x,y,z] = sphere(16);
scatter3(x_traj,y_traj,z_traj,'.');

%=========================================================================
%Trajektoria 3

pohyb3 = KEM.jtraj(P3,P4,50);
q_f3 = KEM.fkine(pohyb3)

x_traj = zeros(1,50);
y_traj = zeros(1,50);
z_traj = zeros(1,50);


for i=1:50
x_traj(1,i) = q_f3(1,4,i);
y_traj(1,i) = q_f3(2,4,i);
z_traj(1,i) = q_f3(3,4,i);
end

hold on
[x,y,z] = sphere(16);
scatter3(x_traj,y_traj,z_traj,'.');

%========================================================================
%Trajektoria 4

pohyb4 = KEM.jtraj(P4,P1,50);
q_f4 = KEM.fkine(pohyb4)

x_traj = zeros(1,50);
y_traj = zeros(1,50);
z_traj = zeros(1,50);

for i=1:50
x_traj(1,i) = q_f4(1,4,i);
y_traj(1,i) = q_f4(2,4,i);
z_traj(1,i) = q_f4(3,4,i);
end

hold on
[x,y,z] = sphere(16);
scatter3(x_traj,y_traj,z_traj,'.');
%=========================================================================
%Simulacia

vstup = 0;

while (vstup ~= 1)
KEM.plot(pohyb1)
KEM.plot(pohyb2)
KEM.plot(pohyb3)
KEM.plot(pohyb4)
end
