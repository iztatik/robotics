clear , close, clc;

            %Creamos un Robot mediante sus parametros D-H

d1 = 1;
d2 = 1;
d6 = 1;

L1=Revolute ('a', 0, 'alpha', 0,'d', d1);    %Eslab�n 1 
L2=Prismatic('a', 0,'alpha', -pi/2,   'theta', -pi/2,'qlim',[0.5,2]); %Eslab�n 3
L3=Prismatic('a', 0,'alpha', 0,   'theta', 0,'qlim',[0.5,2]); %Eslab�n 3
L4=Revolute('a', 0,  'alpha', -pi/2,'d', 0);  %Eslab�n 4 
L5=Revolute('a', 0, 'alpha', pi/2,  'd', 0);  %Eslab�n 5
L6=Revolute('a', 0, 'alpha', 0,     'd', d6); %Eslab�n 6

bot = SerialLink([L1, L2, L3, L4, L5, L6]); %Creamos el robot

    
            %Animaci�n
ws= ([-4 4 -4 4 -1 4]);
%bot.plot([0,0.5,0.5,0,0,0],'workspace',ws,'noname');
bot.teach([0,0.5,0.5,0,0,0],'workspace',ws,'noname')
