clear , close, clc;

            %Creamos un Robot mediante sus parametros D-H

d6 = 0.1;

L4=Revolute('a', 0,  'alpha', -pi/2,'d', 0); %Eslab�n 4 
L5=Revolute('a', 0, 'alpha', pi/2,  'd', 0); %Eslab�n 5
L6=Revolute('a', 0, 'alpha', 0,     'd', d6);%Eslab�n 6

wrist=SerialLink([L4, L5, L6]); %Creamos el robot

    
            %Animaci�n
%bot.plot([pi/4,pi/4,pi/4]);
wrist.teach([0, 0, 0])