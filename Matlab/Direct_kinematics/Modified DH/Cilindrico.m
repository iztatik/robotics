clear ;close ; clc;                    

high=1;
a2=0.3;
l=0.5;

Tb=[1 0 0 0 ;0 1 0 0;0 0 1 high;0 0 0 1];   %trama base
Tt=[1 0 0 0;0 1 0 0;0 0 1 l   ;0 0 0 1];   %Trama herramienta
                        
L1 = RevoluteMDH ('a', 0,    'alpha', 0,    'd',    0);
L2 = PrismaticMDH('a', 0,    'alpha', 0,    'theta',0);
L3 = PrismaticMDH('a',-a2,'alpha', -pi/2,'theta',0);

bot = SerialLink([L1 L2 L3],'base',Tb,'tool',Tt);


                     %Ecuaciones de cinemática inversa
                
Px=[-0.5  -0.5  -0.5  -0.5];            
Py=[1    1    -1    -1];            
Pz=[1.5  2.5  2.5  1.5];

n=size(Px,2);  %Número de puntos en la trayectoria  

for i=1:n
    
    x=Px(i);
    y=Py(i);
    z=Pz(i)-high;
    
    d3(i)= sqrt(x^2+y^2-a2^2)-l;
    d2(i)=z;
    
    phi2(i)= atan(y/x)
    phi1(i)= atan(a2/(d3(i)+l));
    
    if(x<0) phi2(i)=pi+phi2(i);end
    theta1(i)= phi2(i)-phi1(i)-pi/2;
    
end

d3
d2
rad2deg(theta1)

                            
                                    %Animación
t = [0:.05:0.8]';
ws= ([-2 3 -2 2 0 4]);
bot.plot([theta1(1) d2(1) d3(1)],'workspace',ws,'ortho','noname','view', [-130 30]);

for j=1:1
    for i=2:n 
        p1 = [theta1(i-1) d2(i-1) d3(i-1)];
        p2 = [theta1(i) d2(i) d3(i)];
        q = jtraj(p1,p2, t);
        bot.animate(q);
    end
    
    for i=0:n-2
        p1 = [theta1(n-i) d2(n-i) d3(n-i)];
        p2 = [theta1(n-i-1) d2(n-i-1) d3(n-i-1)];
        q = jtraj(p1,p2, t);
        bot.animate(q);
    end
end
                        