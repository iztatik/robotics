clear , close, clc;

            %Creamos un Robot mediante sus parametros D-H

d1 = 1;

L1=Revolute ('a', 0, 'alpha', 0,'d', d1);    %Eslabón 1 
L2=Prismatic('a', 0,'alpha', -pi/2,   'theta', -pi/2,'qlim',[0.5,2]); %Eslabón 2
L3=Prismatic('a', 0,'alpha', 0,   'theta', 0,'qlim',[0.5,2]); %Eslabón 3

bot=SerialLink([L1, L2, L3]); %Creamos el robot

    
            %Animación
t = [0:.05:0.8]';
ws= ([-3 3 -3 3 -1 3]);
%bot.plot([pi/4,pi/4,2],'workspace',ws );
bot.teach([pi/4,1,1],'workspace',ws,'noname')


% C. Directa
P = zeros(3,1);
while (true)
    input('¿Calcular?')
    q = bot.getpos();           

    P(1) = q(3)*cos(q(1));
    P(2) = q(3)*sin(q(1));
    P(3) = d1 + q(2);
    
    disp(P)
end
