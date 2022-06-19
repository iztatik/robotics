clear; close; clc;  

        %Creamos un Robot mediante sus parametros D-H
d1=2;
d2 = 2;
d3=0.1;
a1=1;
a2=1;
l=0.1;

Tb=[1 0 0 0; 
    0 1 0 0; 
    0 0 1 d1; 
    0 0 0 1];   %trama base
Tt=[1 0 0 0; 
    0 1 0 0; 
    0 0 1 l; 
    0 0 0 1];   %Trama herramienta

L1=RevoluteMDH ('a',0, 'alpha', 0,   'd', 0);    %Eslabón 1 
L2=RevoluteMDH ('a',a1, 'alpha', 0,'d', 0);     %Eslabón 2
L3=PrismaticMDH('a',a2, 'alpha', pi,'theta',0); %Eslabón 3

bot = SerialLink([L1 L2 L3],'base',Tb,'tool',Tt); %Creamos el robot


                         %Ecuaciones de cinemática inversa
                
Px=[0.5  -0.5  -0.5  0.5];            
Py=[1    1    -1    -1];            
Pz=[0.5  1.5  1.5  0.5];

n=size(Px,2);  %Número de puntos en la trayectoria  

for i=1:n
    
    x=Px(i);
    y=Py(i);
    z=d1-Pz(i);

    D=(x^2+y^2-a1^2-a2^2)/(2*a1*a2);
    theta2(i) = atan(sqrt(1-D^2)/D); % -=codo arriba,+=codo abajo
    if(D<0)   theta2(i)=-(pi-theta2(i));end 
    theta1(i) = atan(y/x)-atan((a2*sin(theta2(i)))/(a1+a2*cos(theta2(i))));
    if (x<0)   theta1(i)= pi+theta1(i); end
    l(i)=z-d3;
end

theta1
theta2
l
  

                                    %Animación
t = [0:.05:0.8]';
ws= ([-2 3 -2 2 0 4]);
bot.plot([theta1(1) theta2(1) l(1)],'jaxes','workspace',ws,'trail',{'r.','LineWidth',3},'ortho','jaxes','noname','view', [70 15]);

for j=1:1
    for i=2:n 
        p1 = [theta1(i-1) theta2(i-1) l(i-1)];
        p2 = [theta1(i) theta2(i) l(i)];
        q = jtraj(p1,p2, t);
        bot.animate(q);
    end
    
    for i=0:n-2
        p1 = [theta1(n-i) theta2(n-i) l(n-i)];
        p2 = [theta1(n-i-1) theta2(n-i-1) l(n-i-1)];
        q = jtraj(p1,p2, t);
        bot.animate(q);
    end
end