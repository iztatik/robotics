clear; close; clc;

% Variable definitions
syms t;
a0 = pi/6;
a1 = 1;
a2 = 0;
a3 = -3.17275;
a4 = 2.50456;
a5 = -0.51341;

%Polinomyals
theta = a0+(a1*t)+(a2*t^2)+(a3*t^3)+(a4*t^4)+(a5*t^5)
theta_p = diff(theta)
theta_pp = diff(theta_p)
theta_ppp = diff(theta_pp)

% Simulation time
t=linspace(0,2,101);

% Evalution of "defined" t
pos = double(subs(theta));
vel = double(subs(theta_p));
acc = double(subs(theta_pp));
jerk = double(subs(theta_ppp));

% Plots
figure;
title('Position')
plot(t,pos);
grid on;

figure;
title('Velocity')
plot(t,vel);
grid on;

figure;
title('Acceleration')
plot(t,acc);
grid on;


