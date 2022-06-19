clear , close, clc;

            %Creamos un Robot mediante sus parametros D-H

d1 = 2;
d6 = 0.5;

L1=Revolute('a', 0, 'alpha', pi/2,'d', d1);    %Eslab�n 1 
L2=Revolute('a', 0, 'alpha', pi/2,'d', 0);     %Eslab�n 2
L3=Prismatic('a', 0,'alpha', 0,   'theta', 0,'qlim',[0,4]); %Eslab�n 3
L4=Revolute('a', 0,  'alpha', -pi/2,'d', 0);  %Eslab�n 4 
L5=Revolute('a', 0, 'alpha', pi/2,  'd', 0);  %Eslab�n 5
L6=Revolute('a', 0, 'alpha', 0,     'd', d6); %Eslab�n 6

bot=SerialLink([L1, L2, L3,L4,L5,L6]); %Creamos el robot

    
            %Animaci�n
t = [0:.05:0.8]';
ws= ([-8 8 -8 8 -8 8]);
%bot.plot([pi/4,pi/4,2],'workspace',ws );
bot.teach([0,0,1,0,0,0])
