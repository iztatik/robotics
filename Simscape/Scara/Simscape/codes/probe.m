clear; close; clc;
%% Parameters
simTime = 3;

%% Object creation
my_scara = scara(0.5, 0.5, 0.5, 0.1);

%% Trajectory planning
pose = input('Desired pose: [x, y, z, alpha]: ');
vel = input('Initial and final velocites [V0, Vf]: ');
time = input('Time to reach the pose: ');
%q = my_scara.line(pose);
q = my_scara.jtrajCubic(pose, vel, time);

%% Simulation
t = linspace(0,simTime,size(q,1))';

q1 = [t, q(:,1)];
q2 = [t, q(:,2)];
q3 = [t, q(:,3)];
q4 = [t, q(:,4)];

sim('..\scara4dof.slx','TimeOut',simTime);
