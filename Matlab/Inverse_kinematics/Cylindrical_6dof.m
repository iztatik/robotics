clear , close, clc;

prm = param();

%Creamos un Robot mediante sus parametros D-H
L1=Revolute ('a', 0, 'alpha', 0,'d', prm.d1);    %Eslabón 1 
L2=Prismatic('a', 0,'alpha', -pi/2,   'theta', -pi/2,'qlim',[0.5,2]); %Eslabón 3
L3=Prismatic('a', 0,'alpha', 0,   'theta', 0,'qlim',[0.5,2]); %Eslabón 3
L4=Revolute('a', 0,  'alpha', -pi/2,'d', 0);  %Eslabón 4 
L5=Revolute('a', 0, 'alpha', pi/2,  'd', 0);  %Eslabón 5
L6=Revolute('a', 0, 'alpha', 0,     'd', prm.d6); %Eslabón 6

bot = SerialLink([L1, L2, L3, L4, L5, L6]); %Creamos el robot
    
% Pose inicial (home)
home = [0,0.5,0.5,0,0,0];
ws= ([-4.5 4.5 -4.5 4.5 -1 4.5]);
bot.teach(home,'workspace',ws,'noname')

while true
   O = input('Introduza un vector de posición deseada [x,y,z]: ');            % Posición deseada: [3,2,3],[-3,-2,4]
   Rrpy = input('Introduza un vector de orientación deseado rpy[z,y,x]: ');   % Orientación deseada RPY[z, y, x] (grados): [-10,45,-45] 
   
   % 1) Calculamos el centro de la muñeca con la posición y orientación deseadas
   [xc, yc, zc] = wristCenter(O(1), O(2), O(3), Rrpy);
   
   % 2) Aplicamos cinemática inversa al cuerpo para obtener q1, q2, q3
   [q1, q2, q3] = invPos(xc, yc, zc);
   
   % 3) Obtenemos R_0_3 evaluando [q1, q2, q3] en la C. directa del cuerpo
   R_0_3 = bodyOrientation(q1, q2, q3);

   % 4) Obtenemos las variables q4, q5, q6 a partir de los ángulos de Euler
   [q4, q5, q6] = wristOrientation(R_0_3, Rrpy);

    % Evaluamos y graficamos
    plot_sphere(O, 0.25, 'y')
    input('Presione una tecla para mover al robot...')  
    bot.teach([q1, q2, q3, q4, q5, q6],'workspace',ws,'noname')

end


% Centro de la muñeca
function [xc, yc, zc] = wristCenter(ox, oy, oz, Rrpy)
    prm = param();
    
    R = rpy2r(Rrpy,'xyz');
    r13 = R(1,3);
    r23 = R(2,3);
    r33 = R(3,3);
    
    xc = ox - prm.d6*r13;
    yc = oy - prm.d6*r23;
    zc = oz - prm.d6*r33;
end

% Posición inversa (cuerpo)
function [q1, q2, q3] = invPos(x, y, z)  % s: sign of the q2 joint variable
    prm = param();
    
    q1 = atan2(y, x);
    q2 = z - prm.d1;
    q3 = sqrt(x^2 + y^2);
end

% Orientación del cuerpo
function R_b = bodyOrientation(q1, q2, q3)
    
    R_b = [sin(q1), 0, cos(q1);
          -cos(q1), 0, sin(q1);
             0,    -1,   0   ];
end


% Orientación de la muñeca
function [q4, q5, q6] = wristOrientation(R_0_3, Rrpy)
    
     R_3_6 = R_0_3'*rpy2r(Rrpy,'xyz');
     k = tr2eul(R_3_6);
     q4 = k(1);
     q5 = k(2);
     q6 = k(3);
     
end


% Función con parámetros
function g = param(name, value)
    persistent buff;
    
    %Inicializar
    if isempty(buff)
        buff.d1 = 2;
        buff.d6 = 1;
    end
    
    % Sobreescribir valores
    if nargin > 0
        buff.(name) = value;
    end
    
    g = buff;
end
