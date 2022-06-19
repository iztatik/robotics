clear , close, clc;

            %Creamos un Robot mediante sus parametros D-H

d6 = 0.1;

L4=Revolute('a', 0,  'alpha', -pi/2,'d', 0); %Eslabón 4 
L5=Revolute('a', 0, 'alpha', pi/2,  'd', 0); %Eslabón 5
L6=Revolute('a', 0, 'alpha', 0,     'd', d6);%Eslabón 6

wrist=SerialLink([L4, L5, L6]); %Creamos el robot

    
            %Animación
%bot.plot([pi/4,pi/4,pi/4]);
wrist.teach([0, 0, 0])