clear; close; clc;
prm = param();

% Virtual model defined by its D-H parameters
L(1)=Revolute('a',    0,   'alpha',  pi/2, 'd', prm.d1);  %  Linkage 1
L(2)=Revolute('a', prm.a2, 'alpha',   0,   'd',   0);     %  Linkage 2
L(3)=Revolute('a',    0,   'alpha', -pi/2, 'd',   0);     %  Linkage 3
L(4)=Revolute('a',    0,   'alpha', -pi/2, 'd', prm.d4);  %  Linkage 4
L(5)=Revolute('a',    0,   'alpha',  pi/2, 'd',   0);     %  Linkage 5
L(6)=Revolute('a',    0,   'alpha',   0,   'd', prm.d6);  %  Linkage 6

bot = SerialLink(L); %Creamos el robot 

% Initial position (Home)
home = [0,pi/2,-pi/2,0,0,0];
ws = [-726,726,-726,726,0,726];
bot.teach(home, 'workspace', ws, 'noname')

while true
    P = input ('Introduzca un vector de posición deseada [x,y,z]:'); %Posición  [268.372, -268.372, 437.275][327.3, 116, 446.8]
    Rrpy= input('Introduzca un vector de orientación deseado rpy[z,y,x]:'); %Orientacion [-pi/4, 0, 0][1.2741,  0.5236, -1.2217]
    
    %1) Wrist center estimation
    [xc , yc, zc] = wristCenter (P(1),P(2), P(3), Rrpy);
    
    %2) Computation of the body joint variables [q1,q2,q3]
    [q1 , q2 , q3]= invPos (xc , yc ,zc);
    
    %3) Body orientation R_0_3 
    R_0_3 = bodyOrientation(q1 , q2 , q3);
    
    %4) Wrist vjoint variables [q4, q5, q6]
    [q4 , q5 , q6] = wristOrientation(R_0_3, Rrpy);
 
    % Ploting and animation
    plot_sphere(P, 50 ,'y')
    input('Presione una tecla para mover al robot ...')
    bot.teach([q1 , q2 , q3 ,q4, q5 , q6],'workspace',ws,'noname') 
    
end

% Wrist center
function [xc , yc , zc]= wristCenter(ox , oy , oz , Rrpy)
  prm = param();
     
  R = rpy2r(Rrpy, 'xyz');   
  r13=R(1,3);
  r23=R(2,3);
  r33=R(3,3);
  
  xc = ox - (prm.d6)*r13;
  yc = oy - (prm.d6)*r23;
  zc = oz - (prm.d6)*r33;
  
end

%Inverse position (Boydy  inverse kinematics)
function [q1 , q2 , q3] = invPos(xc, yc, zc) 
   prm = param();
   D = -(xc^2 + yc^2 + (zc-prm.d1)^2 - prm.a2^2 - prm.d4^2) / (2*prm.a2*prm.d4);

   q3 = atan2(D, sqrt(1-D^2));
   q2 = atan2(zc-prm.d1, sqrt(xc^2 + yc^2)) - atan2(prm.d4*cos(q3), prm.a2 - prm.d4*sin(q3));
   q1 = atan2(yc, xc);
end

% Body orientation
function R_b = bodyOrientation(q1,q2,q3)

  R_b = [ cos(q1)*cos(q2+q3) , -sin(q1)  , -cos(q1)*sin(q2+q3) ;
          sin(q1)*cos(q2+q3) ,  cos(q1)  , -sin(q1)*sin(q2+q3) ;
             sin(q2+q3)      ,    0      ,     cos(q2+q3)     ];           

end

% Wrist orientation
function [q4 , q5 , q6] = wristOrientation(R_0_3,Rrpy)        
  R_3_6 = (R_0_3)'*rpy2r(Rrpy,'xyz'); 
  k = tr2eul(R_3_6); 
   
  q4= k(1);
  q5= k(2);
  q6= k(3);
end

% Parameters
function g= param(name, value)
   persistent buff;
      %Initialize
   if isempty(buff)
       buff.d1 = 152.75;
       buff.a2 = 221.12;
       buff.d4 = 223.18;
       buff.d6 = 128.17;             
   end      
   % Overwrite vaues
   if nargin > 0       
       buff.(name)= value;
   end   
  g=buff;       
end
