clear , close, clc;

            %Creamos un Robot mediante sus parametros D-H

a1 = 1;
a2 = 1;

L1=Revolute('a', a1, 'alpha', 0,   'd', 0); %Eslabón 1
L2=Revolute('a', a2, 'alpha', 0,   'd', 0); %Eslabón 2

bot=SerialLink([L1, L2]); %Creamos el robot
    
            %Animación
ws= ([-2 2 -2 2 -1 2]);
% bot.plot([pi/2,pi/2]);
bot.teach([0, pi/4],'workspace',ws)


% C. Directa
P = zeros(3,1);
while (true)
    input('¿Calcular?')
    q = bot.getpos();           

    P(1) = a1*cos(q(1)) + a2*cos(q(1)+q(2));
    P(2) = a1*sin(q(1)) + a2*sin(q(1)+q(2));
    
    disp(P)
end



