clear; close; clc;

% Define the poses to represent
Pose0 = SO3;
Pose1 = SO3.Ry(-90);
Pose2 = Pose1*SO3.Rx(90);
%Pose3 = Pose2*SO3.Rz(180);

% Animate movements one-by-one
figure();
trplot(Pose0,'rgb','axis',[-3,3,-3,3,-3,3])
pause;
tranimate(Pose0,Pose1,'rgb','axis',[-3,3,-3,3,-3,3])
pause;
tranimate(Pose1,Pose2,'rgb','axis',[-3,3,-3,3,-3,3])
pause;
% tranimate(Pose2,Pose3,'rgb','axis',[-3,3,-3,3,-3,3])

% Print resulting matrix
disp(Pose2)

% Animate movements in 1 step
close all
figure();
trplot(Pose0,'rgb','axis',[-3,3,-3,3,-3,3])
pause;
tranimate(Pose0,Pose2,'rgb','axis',[-3,3,-3,3,-3,3])
