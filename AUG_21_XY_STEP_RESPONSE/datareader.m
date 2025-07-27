clc;
close;

data = load('./XY_SR_001.csv');

ez = data(:,3);
x = data(:,10);
vx = data(:,4);

y = data(:,15);
vy = data(:,14);
sp = data(:,14);
thrust = data(:,16);
time = 1:1:numel(thrust);


figure(1), plot(time, ez);

figure(2), plot(time, thrust);

figure(3), plot(x,y);
figure(4), plot(time, vx)