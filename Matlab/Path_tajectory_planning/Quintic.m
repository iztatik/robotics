clear; close; clc;

%% Conditions
syms t;
t_f = input('Final time(s): ');
p_0 = input('Initial angular position: ');
p_f = input('Final angular position: ');
v_0 = input('Initial angular velocity: ');
v_f = input('Final angular velocity: ');
a_0 = input('Initial angular acceleration: ');
a_f = input('Final angular acceleration: ');

%% Coefficients calculation
a0 = p_0;
a1 = v_0;
a2 = a_0/2;
a3 = ((20*p_f-20*p_0)-(8*v_f+12*v_0)*t_f+(a_f-3*a_0)*t_f^2)/(2*t_f^3);
a4 = -((30*p_f-30*p_0)-(14*v_f+16*v_0)*t_f+(2*a_f-3*a_0)*t_f^2)/(2*t_f^4);
a5 = ((12*p_f-12*p_0)-(6*v_f+6*v_0)*t_f+(a_f-a_0)*t_f^2)/(2*t_f^5);

disp('Coefficients: ')
disp([a0, a1, a2, a3, a4, a5])

%% Polinomyals
theta = a0+(a1*t)+(a2*t^2)+(a3*t^3)+(a4*t^4)+(a5*t^5);
theta_p = diff(theta);
theta_pp = diff(theta_p);
theta_ppp = diff(theta_pp);

disp('Position: ')
disp(theta)
disp('Velocity: ')
disp(theta_p)
disp('Acceleration: ')
disp(theta_pp)
disp('Jerk: ')
disp(theta_ppp)

%% Simulation time
t=linspace(0,t_f,100);

%% Evalution of "defined" t
pos = double(subs(theta));
vel = double(subs(theta_p));
acc = double(subs(theta_pp));
jerk = double(subs(theta_ppp));

%% Plots
figure;
plot(t,pos);
title('Position')
grid on;

figure;
plot(t,vel);
title('Velocity')
grid on;

figure;
plot(t,acc);
title('Acceleration')
grid on;

figure;
plot(t,jerk);
title('Jerk')
grid on;


