clear; close; clc;

%% Robot descriptor
d1 = 2.5; 
a1 = 1.5; 
a2 = 1.5;
d4 = 1;

L1 = Revolute('a',a1,'alpha',0,'d',d1); 
L2 = Revolute('a',a2,'alpha',pi,'d',0); 
L3 = Prismatic('a', 0,'alpha', 0,'theta',0, 'qlim', [0.5,2]);
L4 = Revolute('a',0,'alpha',0,'d',d4); 

scara = SerialLink([L1,L2,L3,L4]);

%% Plot the robot
ws = [-4,4,-4,4,-1,4];
scara.teach([pi/2,pi/3,1,-pi/5], 'workspace',ws);
