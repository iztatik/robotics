clear; close; clc;

% Importar modelo 
mdl_puma560;

% Posici�n objetivo 
p = [0.7 0 0];
T = transl(p)*troty(pi/2);

% Cinem�tica inversa
qrr = p560.ikine6s(T,'ru');

% Generar un polinomio de grado 5
qrt = jtraj(qr,qrr,50);

% Vista
ae = [138 8];

% Compensando la posici�n
p(1)= p(1)+0.2;

% Animando
plot_sphere(p, 0.05, 'y')
p560.plot3d(qrt, 'view',ae)
