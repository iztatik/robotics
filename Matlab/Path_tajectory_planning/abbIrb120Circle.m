
% Import Robot
clear
clc
robot=loadrobot("abbIrb120");
show(robot);
axes.CameraPositionMode = 'auto';
 % Defines the trajectory as a circle with a radius of 0.15
t = (0:3:70)'; 
count = length(t);
center = [0.45 0 0.3];
radius = 0.15;
theta = t*(2*pi/t(end));
points =(center + radius*[cos(theta) sin(theta) zeros(size(theta))])';
%%
hold on
 plot3 (points (1, :), points (2, :), points (3, :), 'r')% draw the defined trajectory
 % Inverse kinematics solution: use the InverseKinematics object to find a solution for robot configuration to achieve the final end position given along the trajectory
eeOffset = 0.01;
eeBody = robotics.RigidBody('end_effector');
setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
addBody(robot,eeBody,'link_6');
ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0 1 1 1];
qInitial = robot.homeConfiguration;
%%
 % Trace the circle through the trajectory of the point. Call the ik object at each point to generate the joint configuration that implements the end position, and store the configuration to be used later.
for i = 1:size(points,2)
    % Solve for the configuration satisfying the desired end effector
    tform = trvec2tform(points(:,i)');
    qSol(i,:) = ik('end_effector',tform,weights,qInitial);
    % Start from prior solution
    qInitial = qSol(i,:);
end
%%
title('robot move follow the trajectory')
hold on
axis([-0.6 0.8 -0.6 0.65 0 1.3]);
for i = 1:size(points,2)
         show (robot, qSol (i, :)' ,'PreservePlot', false); %When false is changed to true, a ghost image is left.
    pause(3/70)
end
hold off