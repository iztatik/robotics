clear all, close all, clc;

            %Creamos un Robot mediante sus parametros D-H

high=10;
a2=14;
a3=31.35;

Tb=[1 0 0 0 ;
    0 1 0 0;
    0 0 1 high;
    0 0 0 1];   %trama base

Tt=[1 0 0 a3;
    0 1 0 0;
    0 0 1 0 ;
    0 0 0 1];   %Trama herramienta

L1=RevoluteMDH('a', 0,  'alpha', 0,   'd', 0); %Eslabón 1 
L2=RevoluteMDH('a', 0,  'alpha', pi/2,'d', 0); %Eslabón 2
L3=RevoluteMDH('a', a2, 'alpha', 0,   'd', 0); %Eslabón 3

bot=SerialLink([L1 L2 L3],'tool',Tt,'base',Tb); %Creamos el robot


            %Ecuaciones de cinemática inversa
                
Px=[-20  -20  -20  -20];            
Py=[10    10    -10    -10];            
Pz=[15  25  25  15];

n=size(Px,2);  %Número de puntos en la trayectoria  

for i=1:n
    
    x=Px(i);
    y=Py(i);
    z=Pz(i)-high;

    D=(x^2+y^2+z^2-a2^2-a3^2)/(2*a2*a3);
    theta3(i) = atan(sqrt(1-D^2)/D); % -=codo arriba,+=codo abajo
    if(D<0)   theta3(i)=-(pi-theta3(i)) ;end  
    theta2(i) = atan(z/sqrt(x^2+y^2))-atan((a3*sin(theta3(i)))/(a2+a3*cos(theta3(i))))
    theta1(i) = atan(y/x);
    
end
    
            %Animación
t = [0:.05:0.8]';
ws= ([-30 30 -20 20 -30 30]);
bot.plot([theta1(1) theta2(1) theta3(1)],'workspace',ws,'trail',{'r.','LineWidth',3},'ortho','jaxes','noname','tilesize',2.0,'view', [50 30]);

% 
% for j=1:1
%     for i=2:n 
%         p1 = [theta1(i-1) theta2(i-1) theta3(i-1)];
%         p2 = [theta1(i) theta2(i) theta3(i)];
%         q = jtraj(p1,p2, t);
%         bot.animate(q);
%     end
%     
% %     for i=0:n-2
% %         p1 = [theta1(n-i) theta2(n-i) theta3(n-i)];
% %         p2 = [theta1(n-i-1) theta2(n-i-1) theta3(n-i-1)];
% %         q = jtraj(p1,p2, t);
% %         bot.animate(q);
% %     end
% end
% 
% tt =[0.25 0.5 0.75 1];
% figure 
% plot (tt,theta1);
% figure 
% plot (tt,theta2);
% figure 
% plot (tt,theta3);