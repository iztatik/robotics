clear; close; clc;

% Definimos las tramas a representar
Pose0 = SE3;
Pose1 = SE3.Ry(-90);
Pose2 = Pose1*SE3(0,0,-2);
Pose3 = Pose2*SE3.Rx(90);
Pose4 = Pose3*SE3(-2,0,0);

figure()

trplot(Pose0,'rgb','axis',[-3,3,-3,3,-3,3])
pause;
tranimate(Pose0,Pose1,'rgb','axis',[-3,3,-3,3,-3,3])
hold on
tranimate(Pose1,Pose2,'rgb','axis',[-3,3,-3,3,-3,3])
hold on
tranimate(Pose2,Pose3,'rgb','axis',[-3,3,-3,3,-3,3])
hold on
tranimate(Pose3,Pose4,'rgb','axis',[-3,3,-3,3,-3,3])

H =  SE3.Ry(-pi/2)*SE3(0,0,-2)*SE3.Rx(pi/2)*SE3(0,0,-1)

pause;
close all
figure()
pause;
tranimate(Pose0,H,'rgb','axis',[-3,3,-3,3,-3,3])