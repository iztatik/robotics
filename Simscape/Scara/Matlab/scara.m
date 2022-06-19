classdef scara < handle
   properties
       % D-H parameters
       d1 {mustBeNumeric}
       a1 {mustBeNumeric}
       a2 {mustBeNumeric}
       d4 {mustBeNumeric}
       
       % Home
       home {mustBeNumeric}
       
       % Robot parameters
       bot
       jointSpace {mustBeNumeric}
       workSpace  {mustBeNumeric}
   end
   
   methods
       % Constructor
       function obj = scara(d1, a1, a2, d4)
           if nargin == 4              
               % D-H parameters
               obj.d1 = d1;
               obj.a1 = a1;
               obj.a2 = a2;
               obj.d4 = d4;
               
               % Home
               obj.home = [0, 0, 0, 0];
               
               % URDF
               obj.bot = importrobot('../urdf/scaraURDF.urdf');
               obj.show(obj.home,false);
                              
               obj.jointSpace = obj.home;
               obj.workSpace = obj.invKin(obj.home);
               
           else
               warning('Insuficeint arguments')
           end
       end
       
       % Show the robot
       function a =show(obj, q, ghost)
           if nargin < 3
               ghost = false;
           end
           % Joint space must be an structure
           q = struct('JointName',{'j1', 'j2', 'j3', 'j4'},'JointPosition',{q(1), q(2), q(3), q(4)});
           show(obj.bot, q ,'PreservePlot', ghost);
       end
       % Inverse kinematics function
        function q = invKin(obj, pose)
            % Pose
            Ox = pose(:,1);
            Oy = pose(:,2);
            Oz = pose(:,3);
            Oa = deg2rad(pose(:,4));
            
            % Equations
            D = (Ox.^2+Oy.^2-obj.a1^2-obj.a2^2)/(2*obj.a1*obj.a2);
            q3 = obj.d1-obj.d4-Oz;
            q2 = atan2(sqrt(1-D.^2), D);
            q1 = atan2(Oy, Ox) - atan2(obj.a2*sin(q2),obj.a1+obj.a2*cos(q2));
            q4 = q1 + q2 - atan2(sin(Oa), cos(Oa));
            
            % Results
            q = [q1,q2,q3,q4];
            
            %Change current spaces
            obj.jointSpace = [q(end,1), q(end,2), q(end,3), q(end,4)];
            obj.workSpace = [pose(end,:)];
        end
        
       % Tace a line
       function l=line(obj, p1, time)
           stp = 100;
           
           % Parametric equations
           p0 = obj.workSpace;
           p0 = [0.3,0.5,0.1,0]
           v = p1-p0;
           a = linspace(p0(4), p1(4), stp);
           x = linspace(p0(1), p1(1),stp);
           
           t = (x-p0(1))/v(1);
           y = p0(2) + v(2)*t;
           z = p0(3) + v(3)*t;
           
           % Results
           l = obj.invKin([x', y', z', a']); 
       end
   end
end