clear; close; clc;

%% Instance the robot
my_elbow = elbow(1.0,1.0,1.0,0.5);

while true
    %% Menu
    list = {'Exit', 'Home', 'Joint trajectory', 'Line  trajectory'};
    m = listdlg('ListString', list);
    if m==1 
        break
    end
    
    %% User data
    if m ~= 2
        Ot = input('Tool position [Ox, Oy, Oz]:' );
        Rt = input('Tool orientation in deg [R, P, Y]: ');
    end

    %% Trajectory planing
    if m == 2
        disp('Selected -> Home... ')
        qTraj = my_elbow.jtraj(my_elbow.home(1:3), my_elbow.home(4:end), [0,0], 2);
    elseif m == 3
        disp('Selected -> Joint trajectory... ')
        qTraj = my_elbow.jtraj(Ot, Rt, [0,0], 2);
    elseif m == 4
        disp('Selected -> Cartesian trajectory... ')
        qTraj = my_elbow.line(Ot, Rt, 2);
    end
        
    %% Plot
    plot_sphere(Ot, 0.2, 'b')
    pause()
    my_elbow.dispRobot(qTraj, 50/3);
end