clear; close; clc;

h = zeros(4,4,3);
h(:,:,1) = transl(0,0,0);
h(:,:,2) = transl(1,1,1)*trotx(90);
h(:,:,3) = transl(0.5,0,0.5)*troty(-90);

figure()
trplot(h(:,:,1),'rgb' ,'axis',[-3,3,-3,3,0,3])
pause()
for i= 1:size(h,3)-1
    tranimate(h(:,:,i),h(:,:,i+1),'rgb','axis',[-3,3,-3,3,0,3])
    pause()
end