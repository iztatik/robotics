clear; close; clc;

% Definimos las tramas a representar
Pose0 = SO3;
Pose1 = SO3.Ry(-90);
Pose2 = Pose1*SO3.Rx(90);

% Lienzo
figure();

% Tramas originales
trplot(Pose0,'axis',[-3,3,-3,3,-3,3])
pause;
hold on
tranimate(Pose0,Pose2,'rgb','axis',[-3,3,-3,3,-3,3],'noxyz')

% Eliminación de la primera rotación mediante su inversa
Pose3 = Pose1.inv()*Pose2*Pose1;
pause;
tranimate(Pose2,Pose3,'rgb','axis',[-3,3,-3,3,-3,3],'noxyz')
