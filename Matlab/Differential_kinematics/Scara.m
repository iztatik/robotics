clear; close; clc;

%% Parameters
d1 = 1;
a1 = 1;
a2 = 1;

%% Variables
% Position
th1 = sym('th1');
th2 = sym('th2');
d3 = sym('d3');

%% Jacobian
J = [-a1*sin(th1)-a2*sin(th1+th2), -a2*sin(th1+th2),  0;
      a1*cos(th1)-a2*cos(th1+th2),  a2*cos(th1+th2),  0;
                 0               ,       0         , -1;
                 0               ,       0         ,  0;
                 0               ,       0         ,  0;
                 1               ,       1         ,  0];
Jv = J(1:3,:);
Jw = J(4:6,:);

disp('The Jacobian is:');
disp(J);
disp('The tangential Jacobian is:');
disp(Jv);
disp('The angular Jacobian is:');
disp(Jw);

%% Inverse of the tangential Jacobian
Jv_inv = simplify(inv(Jv));

disp('The inverse tangential Jacobian is:');
disp(Jv_inv);

%% Evaluation
th1 = input('Angle of joint 1: ');
th2 = input('Angle of joint 2: ');
d3 = input('Distance of joint 3: ');

th1_p = input('Speed of joint 1: ');
th2_p = input('Speed of joint 2: ');
d3_p = input('Speed of joint 3: ');

q_p = [th1_p, th2_p, d3_p]';

%% Direct Differential Kinematics (DDK)
V = subs(Jv)*q_p;
W = subs(Jw)*q_p;

disp('The Cartesian velocity vector is:');
disp(V);
disp('Approximately:');
disp(double(V));

disp('The angular velocity vector is:');
disp(W);
disp('Approximately:');
disp(double(W));

%% Inverse Differential Kinematics (IDK)
q_p = subs(Jv_inv)*V;

disp('The Joint velocity vector is:');
disp(q_p);
disp('Approximately:');
disp(double(q_p));
