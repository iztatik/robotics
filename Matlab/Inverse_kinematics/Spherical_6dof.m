clear , close, clc;

prm = param();
            %Creamos un Robot mediante sus parametros D-H

L1=Revolute('a', 0, 'alpha', pi/2,'d', prm.d1);    %Eslab�n 1 
L2=Revolute('a', 0, 'alpha', pi/2,'d', 0);     %Eslab�n 2
L3=Prismatic('a', 0,'alpha', 0,   'theta', 0,'qlim',[0,4]); %Eslab�n 3
L4=Revolute('a', 0,  'alpha', -pi/2,'d', 0);  %Eslab�n 4 
L5=Revolute('a', 0, 'alpha', pi/2,  'd', 0);  %Eslab�n 5
L6=Revolute('a', 0, 'alpha', 0,     'd', prm.d6); %Eslab�n 6

bot=SerialLink([L1, L2, L3,L4,L5,L6]); %Creamos el robot

    
% Pose inicial
          
ws= ([-6 6 -6 6 -1 6]);
bot.teach([0,0,1,0,0,0])


% Funci�n con par�metros
function g = param(name, value)
    persistent buff;
    
    %Inicializar
    if isempty(buff)
        buff.d1 = 2;
        buff.d6 = 0.5;
    end
    
    % Sobreescribir valores
    if nargin > 0
        buff.(name) = value;
    end
    
    g = buff;
end
