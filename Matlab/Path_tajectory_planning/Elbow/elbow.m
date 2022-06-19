classdef elbow < handle
   properties
       % D-H parameters
       d1 {mustBeNumeric}
       a2 {mustBeNumeric}
       d4 {mustBeNumeric}
       d6 {mustBeNumeric}
       
       % Robot      
       bot
       
       % Positions
       home {mustBeNumeric}
       qPos {mustBeNumeric}
       cPos {mustBeNumeric}
       
       
   end
   
   methods
       % Constructor
       function obj = elbow(d1,a2,d4,d6)
           if nargin == 4
               obj.d1 = d1;
               obj.a2 = a2;
               obj.d4 = d4;
               obj.d6 = d6;
               
               % Positions
               obj.home = [0, 0, 0, 0, 0, 0];
               obj.qPos = obj.home;
               obj.cPos = [1, 0, 2.5, 0, 0, 0];
               
               % Define and plot the robot
               obj.robotDef()
           else
               warning('Insuficient arguments')
           end
       end
       
       % Robot definition
       function robotDef(obj)
           % D-H parameters
           L(1) = Revolute('a',0,'alpha',pi/2,'d',obj.d1);
           L(2) = Revolute('a',obj.a2,'alpha',0,'d',0);
           L(3) = Revolute('a',0,'alpha',-pi/2,'d',0);
           L(4) = Revolute('a',0,'alpha',-pi/2,'d',obj.d4);
           L(5) = Revolute('a',0,'alpha',pi/2,'d',0);
           L(6) = Revolute('a',0,'alpha',0,'d',obj.d6);
           
           % Robot building
           obj.bot = SerialLink(L);
                     
           % Plot the robot
           obj.dispRobot(obj.home, Inf);
       end
       
       % Plot the robot
       function dispRobot(obj, q, fps)
           % Plot the robot
           ws = [-3,3,-3,3,0,5];
           obj.bot.plot(q,'workspace',ws, 'trail', '.', 'fps', fps);
       end
       
       % Total inverse kinematics
       function q = invKin(obj, Ot, Rt)
           % Wrist position
           Oc = obj.wristCenter(Ot,Rt);
           
           % Inverse position (Body)
           qB = obj.bodyIk(Oc);
           
           % Body´s orientation
           Rb = obj.bodyOri(qB);

           % Wrist angles
           qW = obj.wristOri(Rb, Rt);

           % Total inverse kinematics
           q = [qB, qW];
       end
       
       % Wrist center position
       function Oc = wristCenter(obj, Ot, Rt)
           % Convert rpy to matrix
           Rt = rpy2r(Rt, 'xyz');
           
           % Compute the wrist center
           Xc = Ot(1) - obj.d6*Rt(1,3);
           Yc = Ot(2) - obj.d6*Rt(2,3);
           Zc = Ot(3) - obj.d6*Rt(3,3);
           
           % Return the wrist center
           Oc = [Xc, Yc, Zc];
       end
       
       % Body´s inverse kinematics
       function qB = bodyIk(obj, Oc)
           x = Oc(1);
           y = Oc(2);
           z = Oc(3);
           
           D = -(x^2+y^2+(z-obj.d1)^2-obj.a2^2-obj.d4^2)/(2*obj.a2*obj.d4);
           
           q3 = atan2(D,sqrt(1-D^2));
           q2 = atan2(z-obj.d1, sqrt(x^2+y^2))-atan2(obj.d4*cos(q3), obj.a2-obj.d4*sin(q3));
           q1 = atan2(y,x);
          
           qB = [q1, q2, q3];
       end
       
       % Body´s orientation
       function Rb = bodyOri(obj, qB)
           q1 = qB(1);
           q2 = qB(2);
           q3 = qB(3);
           
           Rb = [cos(q1)*cos(q2+q3),  -sin(q1),  -cos(q1)*sin(q2+q3);
                 sin(q1)*cos(q2+q3),   cos(q1),  -sin(q1)*sin(q2+q3);
                      sin(q2+q3)   ,     0    ,       cos(q2+q3)   ];
           
       end
       
       % Wrist orientation
       function qW = wristOri(obj, Rb, Rt)
           % Convert rpy to matrix
           Rt = rpy2r(Rt, 'xyz');
           
           % Tool orientation with respect to the wrist
           R3_6 = Rb'*Rt;
           
           % Wrist angles
           q4 = atan2(R3_6(2,3),R3_6(1,3));
           q5 = atan2(sqrt(1-R3_6(3,3)^2), R3_6(3,3));
           q6 = atan2(R3_6(3,2),-R3_6(3,1));

           qW = [q4, q5, q6];
       end
       
       %Trajectory planning in the joint space
       function q = jtraj(obj, Ot, Rt, vel, tf)
           % Parameters
           stp = 50;
           
           % Variables
           syms t
            
           % Requirements
           q0 = obj.qPos;
           qf = obj.invKin(Ot, Rt);
           if sum([Ot, Rt]) == 0
               qf = obj.home;
           end
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
           q = double(subs(q, t));
           
           % Replace position
           obj.cPos = [Ot, Rt];
           obj.qPos = qf;
            
       end
        
       % Tace a line
       function l = line(obj, Otf, Rtf, time)
           stp = 50;
           Ot0 = obj.cPos(1:3);
           Rt0 = obj.cPos(3:end);
           
           % Parametric equations
           v = Otf-Ot0;
           x = linspace(Ot0(1), Otf(1), stp);
           r = linspace(Rt0(1), Rtf(1), stp);
           p = linspace(Rt0(2), Rtf(2), stp);
           ya = linspace(Rt0(3), Rtf(3), stp);
           
           
           t = (x-Ot0(1))/v(1);
           y = Ot0(2) + v(2)*t;
           z = Ot0(3) + v(3)*t;
           
           l = zeros(length(x) ,6);
           
           % Results
           for i=1:length(x)
               l(i,:) = obj.invKin([x(i), y(i), z(i)], [r(i), p(i), ya(i)]); 
           end
           
           % Replace position
           obj.cPos = [Otf, Rtf];
           obj.qPos = l(end,:);
       end
       
   end   
end