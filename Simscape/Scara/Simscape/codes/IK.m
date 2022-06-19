clear , close, clc;

prm = param();
simTime = 3; % seconds
stp = 50;

%% Get the desired pose
pose = input('Write the desired pose: [x, y, z, alpha]: ');
q = invkin(pose);

%% Run the simulation
t = linspace(0, simTime, stp)';

q1 = [t,linspace(0, q(1), stp)'];
q2 = [t,linspace(0, q(2), stp)'];
q3 = [t,linspace(0, q(3), stp)'];
q4 = [t,linspace(0, q(4), stp)'];
disp(q)
sim('..\scara4dof.slx','TimeOut',simTime);

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
    q3 = prm.d1 -prm.d4 - z;
    
    q4 = q1 + q2 - atan2(sin(a),cos(a));
    
    k = [q1, q2, q3, q4];
end

%% Parameters
function g = param(name, value)
    persistent buff;
    
    %Inicializar
    if isempty(buff)
        % Units in meters
        buff.d1 = 0.5;
        buff.a1 = 0.5;
        buff.a2 = 0.5;
        buff.d4 = 0.1;
    end
    
    % Sobreescribir valores
    if nargin > 0
        buff.(name) = value;
    end
    
    g = buff;
end