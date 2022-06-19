classdef scara < handle
   properties
       d1 {mustBeNumeric}
       a1 {mustBeNumeric}
       a2 {mustBeNumeric}
       d4 {mustBeNumeric}
       
       jointSpace {mustBeNumeric}
       workSpace {mustBeNumeric}
   end
   
   methods
       % Constructor
       function obj = scara(d1, a1, a2, d4)
           if nargin == 4              
               obj.d1 = d1;
               obj.a1 = a1;
               obj.a2 = a2;
               obj.d4 = d4;
               
               home = [0, 0, 0, 0];
               obj.jointSpace = home;
               obj.workSpace = obj.invKin(home);
           else
               warning('Insuficeint arguments')
           end
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
        
        %Trajectory planning in the joint space
        function q = jtrajCubic(obj, p1, vel, tf)
            % Parameters
            stp = 100;
            
            % Variables
            syms t
            
            % Requirements
            q0 = [0 0 0 0];
            qf = obj.invKin(p1);
            v0 = vel(1);
            vf = vel(2);
            
            % Coefficients calculation
            c0 = q0;
            c1 = v0;
            c2 = (3*(qf-q0)/tf^2)-((vf+2*v0)/tf);
            c3 = (-2*(qf-q0)/tf^3)+ (vf+v0)/tf^2;

            % Polynomia
            q = c0 + c1*t + c2*t^2 + c3*t^3;
            
            % Evaluating the polynomia
            t = linspace(0, tf, stp)';
            q = double(subs(q));
            
        end
        
       % Tace a line
       function l=line(obj, p1, time)
           stp = 100;
           
           % Parametric equations
           p0 = obj.workSpace;
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