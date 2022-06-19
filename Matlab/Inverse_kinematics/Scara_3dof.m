clear , close, clc;

prm = param();

% Creamos un Robot mediante sus parametros D-H
L1 = Revolute('a', prm.a1, 'alpha', 0,'d', prm.d1);    %Eslabón 1 
L2 = Revolute('a', prm.a2, 'alpha', pi,'d', 0);          %Eslabón 2
L3 = Prismatic('a', 0,'alpha', 0,   'theta', 0,'qlim',[0.5,2]); %Eslabón 3

bot = SerialLink([L1, L2, L3]); %Creamos el robot

    
% Animación en la posición home
home = [0, 0, 0.5];    % [2, 0, 1.5] Cartesiano
ws = ([-4 4 -4 4 -1 3]);
bot.teach(home ,'workspace', ws, 'noname')

% bucle para cambiar manualmente la posición
while true
    P = input("Introduza un vector de posición deseada [x,y,z]: ");
    plot_sphere(P, 0.2, 'y')        % [1, 1.5, 0.5], [-1, 1.3, 1], [2, 0, 1.5], [-2, 4, 1]
    input('Presione una tecla para mover al robot')    
    bot.teach(invkin(P(1), P(2), P(3)),'workspace', ws, 'noname')
end


% Cinemática inversa
function k = invkin(x, y, z)  % s: sign of the q2 joint variable
    prm = param();
    
    D = (x^2 + y^2 - prm.a1^2 - prm.a2^2) / (2*prm.a1*prm.a2);

    q2 = atan2(sqrt(1-D^2), D);
    q1 = atan2(y, x) - atan2(prm.a2*sin(q2), prm.a1 + prm.a2*cos(q2));
    q3 = prm.d1 - z;
    k = [q1, q2, q3];
end

% Función con parámetros
function g = param(name, value)
    persistent buff;
    
    %Inicializar
    if isempty(buff)
        buff.a1 = 1;
        buff.a2 = 1;
        buff.d1 = 2;
    end
    
    % Sobreescribir valores
    if nargin > 0
        buff.(name) = value;
    end
    
    g = buff;
end