clear , close, clc;

prm = param();

%% Creamos un Robot mediante sus parametros D-H
L1 = Revolute('a', prm.a1, 'alpha', 0,'d', prm.d1);    
L2 = Revolute('a', prm.a2, 'alpha', pi,'d', 0);          
L3 = Prismatic('a', 0,'alpha', 0,   'theta', 0,'qlim',[0.5,2]); 
L4 = Revolute('a', 0, 'alpha', pi,'d', prm.d4);

bot = SerialLink([L1, L2, L3, L4]); %Creamos el robot

    
%% Animación en la posición home
home = [0, 0, 0.5, 0];    
ws = ([-4 4 -4 4 -1 3]);
bot.teach(home ,'workspace', ws, 'noname')

%% bucle para cambiar manualmente la posición
while true
    P = input("Introduza un vector de posición deseada [x,y,z, deg]: ");
    plot_sphere(P(1:3), 0.2, 'y')        % [1, 1.5, 0.5], [-1, 1.3, 1], [2, 0, 1.5], [-2, 4, 1]
    input('Presione una tecla para mover al robot')    
    bot.teach(invkin(P),'workspace', ws, 'noname')
    disp(invkin(P))
end


%% Inverse kinematics
function k = invkin(pose)  % s: sign of the q2 joint variable
    prm = param();
    x = pose(1);
    y = pose(2);
    z = pose(3);
    a = deg2rad(pose(4));
  
    D = (x^2 + y^2 - prm.a1^2 - prm.a2^2) / (2*prm.a1*prm.a2);

    q2 = atan2(sqrt(1-D^2), D);
    q1 = atan2(y, x) - atan2(prm.a2*sin(q2), prm.a1 + prm.a2*cos(q2));
    q3 = prm.d1 - prm.d4 - z;
    
    q4 = q1 + q2 - atan2(sin(a),cos(a));
    
    k = [q1, q2, q3, q4];
end

% Función con parámetros
function g = param(name, value)
    persistent buff;
    
    %Inicializar
    if isempty(buff)
        % Units in meters
        buff.d1 = 2;
        buff.a1 = 1;
        buff.a2 = 1;
        buff.d4 = 0.02;
    end
    
    % Sobreescribir valores
    if nargin > 0
        buff.(name) = value;
    end
    
    g = buff;
end