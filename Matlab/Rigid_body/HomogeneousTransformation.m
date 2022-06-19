clear; close; clc;

H_0 = SE3();
H_1 = SE3(roty(-90),[2, 0, 0]);
H_2 = SE3(rotx(90),[0, 1, 0]);
H_2_0 = H_1*H_2

% Lienzo
figure();

% Movimiento por partes
trplot(H_0,'rgb','axis',[-2,2,-2,2,-2,2])
hold on
pause;
tranimate(H_0,H_1,'rgb','axis',[-2,2,-2,2,-2,2])
pause;
tranimate(H_1,H_2_0,'rgb','axis',[-2,2,-2,2,-2,2])
pause;


% Movimiento compuesto
hold off
trplot(H_0,'rgb','axis',[-2,2,-2,2,-2,2])
pause;
tranimate(H_0,H_2_0,'rgb','axis',[-2,2,-2,2,-2,2])
