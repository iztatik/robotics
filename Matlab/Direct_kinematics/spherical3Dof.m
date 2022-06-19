clear , close, clc;

            %Creamos un Robot mediante sus parametros D-H

d1 = 2;

L1=Revolute('a', 0, 'alpha', pi/2,'d', d1);    %Eslab�n 1 
L2=Revolute('a', 0, 'alpha', -pi/2,'d', 0);     %Eslab�n 2
L3=Prismatic('a', 0,'alpha', 0,   'theta', 0,'qlim',[0,4]); %Eslab�n 3

bot=SerialLink([L1, L2, L3]); %Creamos el robot

    
            %Animaci�n
ws= ([-5 5 -5 5 -2 5]);
%bot.plot([pi/4,pi/4,2],'workspace',ws );
bot.teach([pi/4,pi/4,2],'workspace',ws,'noname')
