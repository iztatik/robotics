clear all, close all, clc  

        %Creamos un Robot mediante sus parametros D-H

d1=2;
l=0.5;

Tb=[1 0 0 0; 
    0 1 0 0;
    0 0 1 d1; 
    0 0 0 1];   %trama base

Tt=[1 0 0 0; 
    0 1 0 0; 
    0 0 1 l; 
    0 0 0 1];   %Trama herramienta

L1=RevoluteMDH ('a',0, 'alpha', 0,   'd', 0);    %Eslabón 1 
L2=RevoluteMDH ('a',0, 'alpha', pi/2,'d', 0);     %Eslabón 2
L3=PrismaticMDH('a',0, 'alpha', pi/2,'theta',0); %Eslabón 3

bot = SerialLink([L1 L2 L3],'base',Tb,'tool',Tt); %Creamos el robot



   %Ecuaciones de cinemática inversa
                
Px=[-0.5  -0.5  -0.5  -0.5];            
Py=[1    1    -1    -1];            
Pz=[1.5  2.5  2.5  1.5];

n=size(Px,2);  %Número de puntos en la trayectoria  

for i=1:n
    
    x=Px(i);
    y=Py(i);
    z=Pz(i)-d1;
    
    theta1(i)= atan(y/x);
    if (x<0)theta1(i)= pi+atan(y/x); end %+ pues el signo menos ya lo implica el ángulo
    theta2(i)= atan(z/sqrt(x^2+y^2))+pi/2;
    d3(i)= sqrt(x^2+y^2+z^2)-l;
    
end

rad2deg(theta1)
rad2deg(theta2)
d3

                                    %Animación
t = [0:.05:0.8]';
ws= ([-2 3 -2 2 0 4]);
bot.plot([theta1(1) theta2(1) d3(1)],'workspace',ws,'trail',{'r.','LineWidth',3},'ortho','noname','view', [-130 30]);


for j=1:1
    for i=2:n 
        p1 = [theta1(i-1) theta2(i-1) d3(i-1)];
        p2 = [theta1(i) theta2(i) d3(i)];
        q = jtraj(p1,p2, t);
        bot.animate(q);
    end
    
    for i=0:n-2
        p1 = [theta1(n-i) theta2(n-i) d3(n-i)];
        p2 = [theta1(n-i-1) theta2(n-i-1) d3(n-i-1)];
        q = jtraj(p1,p2, t);
        bot.animate(q);
    end
end