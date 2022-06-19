clear , close, clc;

%% Robot definition by its D-H parameters
a1 = 1;

L1=Revolute('a', a1, 'alpha', 0,   'd', 0); %Eslabón 1
bot=SerialLink([L1]);

%% Home position plot
home = [0];    					%Cartesian
ws = ([-4 4 -4 4 -1 3]);
bot.teach(home ,'workspace', ws, 'noname')

%% Conditions
syms t;
t_f = input('Final time(s): ');
p_0 = input('Initial angular position: ');
p_f = input('Final angular position: ');
v_0 = input('Initial angular velocity: ');
v_f = input('Final angular velocity: ');

%% Coefficients calculation
a0 = p_0;
a1 = v_0;
a2 = ((3*p_f-3*p_0)-(v_f+2*v_0)*t_f)/(t_f^2);
a3 = ((-2*p_f+2*p_0)+(v_f+v_0)*t_f)/(t_f^3);

disp('Coefficients: ')
disp([a0, a1, a2, a3])

%% Polinomyals
theta = a0+(a1*t)+(a2*t^2)+(a3*t^3);
theta_p = diff(theta);
theta_pp = diff(theta_p);

disp('Position: ')
disp(theta)
disp('Velocity: ')
disp(theta_p)
disp('Acceleration: ')
disp(theta_pp)

%% Simulation time
stps = 100;
t=linspace(0,t_f,stps);

%% Evaluation of "defined" t
pos = double(subs(theta));
vel = double(subs(theta_p));
acc = double(subs(theta_pp));

%% Animation
disp('Animate?')
pause()
ae = [0 90];
bot.plot(deg2rad(pos)','view',ae,'fps',round(stps/t_f))
