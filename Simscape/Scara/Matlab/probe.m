clear; close; clc;
%% Parameters
simTime = 3;

%% Object creation
my_scara = scara(.5,.5,.5,.1);



%% Trajectory planning
pose = input('Write the desired pose: [x, y, z, alpha]: ');
q = my_scara.line(pose);

%% Simulation
t = linspace(0,simTime,size(q,1))';

q1 = [t, q(:,1)];
q2 = [t, q(:,2)];
q3 = [t, q(:,3)];
q4 = [t, q(:,4)];

%sim('..\scara4dof.slx','TimeOut',simTime);
