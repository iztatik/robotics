clear , close, clc;

            %Creamos un Robot mediante sus parametros D-H

a1 = 1;
a2 = 1; 
d1 = 2;

L1 = Revolute('a', a1, 'alpha', 0,'d', d1);    %Eslabón 1 
L2 = Revolute('a', a2, 'alpha', pi,'d', 0);     %Eslabón 2
L3 = Prismatic('a', 0,'alpha', 0,   'theta', 0,'qlim',[0.5,2]); %Eslabón 3

bot = SerialLink([L1, L2, L3]); %Creamos el robot

    
% Animación
ws = ([-4 4 -4 4 -1 3]);
%bot.plot([pi/4,pi/4,2],'workspace',ws );
bot.teach([0,0,0.5],'workspace', ws, 'noname')


% C. Directa
P = zeros(3,1);
while (true)
    input('¿Calcular?')
    q = bot.getpos();       

    P(1) = a1*cos(q(1)) + a2*cos(q(1)+q(2));
    P(2) = a1*sin(q(1)) + a2*sin(q(1)+q(2));
    P(3) = d1 - q(3);
    
    disp(P)
end