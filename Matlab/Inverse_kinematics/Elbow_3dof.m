clear , close, clc;
prm = param();

            %Creamos un Robot mediante sus parametros D-H

L1=Revolute('a', 0,  'alpha', pi/2,'d', prm.d1); %Eslab�n 1 
L2=Revolute('a', prm.a2, 'alpha', 0,   'd', 0); %Eslab�n 2
%L3=Revolute('a', prm.a3, 'alpha', 0,   'd', 0); %Eslab�n 3
L3 = Link('a', 0, 'alpha', -pi/2,   'd', 0, 'theta', 0);

bot=SerialLink([L1, L2, L3]); %Creamos el robot

   
% Animaci�n en la posici�n home
home = [0, 0, 0.5];    % [2, 0, 1.5] Cartesiano
ws = ([-4 4 -4 4 -1 3]);
bot.teach(home ,'workspace', ws, 'noname')

% Bucle para cambiar manualmente la posici�n
while true
    P = input("Introduza un vector de posici�n deseada [x,y,z]: ");
    plot_sphere(P, 0.2, 'y')        % [1, 1.5, 0.5], [-1, 1.3, 1], [2, 0, 1.5], [-2, 4, 1]
    input('Presione una tecla para mover al robot')    
    bot.teach(invkin(P(1), P(2), P(3)),'workspace', ws, 'noname')
end

% Cinem�tica inversa
function k = invkin(x, y, z)  % s: sign of the q2 joint variable
    prm = param();
    
    D = (x^2 + y^2 + (z-prm.d1)^2 - prm.a2^2 - prm.a3^2) / (2*prm.a2*prm.a3)

    q3 = atan2(sqrt(1-D^2), D);
    q2 = atan2(z-prm.d1, sqrt(x^2 + y^2)) - atan2(prm.a3*sin(q3), prm.a2 + prm.a3*cos(q3));
    q1 = atan2(y, x);
    k = [q1, q2, q3]
end

% Funci�n con par�metros
function g = param(name, value)
    persistent buff;
    
    %Inicializar
    if isempty(buff)
        buff.d1 = 1;
        buff.a2 = 1;
        buff.a3 = 1;
    end
    
    % Sobreescribir valores
    if nargin > 0
        buff.(name) = value;
    end
    
    g = buff;
end