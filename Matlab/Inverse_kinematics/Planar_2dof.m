clear , close, clc;

            %Creamos un Robot mediante sus parametros D-H

a1 = 1;
a2 = 1;

L1=Revolute('a', a1, 'alpha', 0,   'd', 0); %Eslabón 2
L2=Revolute('a', a2, 'alpha', 0,   'd', 0); %Eslabón 3

bot=SerialLink([L1, L2]); %Creamos el robot
    
            %Animación
t = [0:.05:0.8]';
ws= ([-30 30 -20 20 -30 30]);
%bot.plot([pi/2,pi/2]);
bot.teach()

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