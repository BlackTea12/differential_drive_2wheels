clc; close all; clear all;

pose.current = [0 0 0];
goal = [2 2 deg2rad(45)];
r = 0.06; L = 0.6; dt = 1/20;
limit_vel = pi;

% plot setup
figure(); 
plot(0,0,'xb'); grid on; hold on;
xlim([-3 3]); ylim([-3 3]);

for i=0:10/dt
    % sensor value and then
    % error making
    error = goal - pose.current;
    x_diff = error(1);
    y_diff = error(2);
    theta_diff = error(3);
    
    % velocity command output
    [v, w] = DifferentialDriveControl(x_diff, y_diff, theta_diff);
    disp(v); 
    disp(rad2deg(w));
    % wheel command seperation
    [wr, wl] = WheelVelControlSeperation(v, w, r, L);
    
    % into kinematic model to get next postiion
    [xdot, ydot, thetadot] = DifferentialDrive2WheelKinematicModel(pose.current(3), wr, wl, r, L);
    pose.current = pose.current + [xdot, ydot, thetadot]*dt;
    
    plot(pose.current(1), pose.current(2), 'bo');
end

function [xdot, ydot, thetadot] = DifferentialDrive2WheelKinematicModel(theta, wr, wl, r, L)
vx = r * (wr + wl) / 2;     % linear robot's longitudinal velocity
xdot = vx * cos(theta);
ydot = vx * sin(theta);
thetadot = r / L * (wr - wl);
end

function [wr, wl] = WheelVelControlSeperation(v, w, r, L)
wr = (2*v + w*L)/(2*r);
wl = (2*v - w*L)/(2*r);
end

function [v, w] = DifferentialDriveControl(x_diff, y_diff, theta_diff)
% polar coordinate values
alpha = theta_diff;
rou = sqrt(x_diff^2 + y_diff^2);

% tuning value
Kp = 3;
Ka = 6;
h = 1;

% output
v = Kp*rou*cos(alpha);
w = Kp*sin(alpha)*cos(alpha) + Ka*alpha;
end

function Vec2D(x,y,theta,size)
xu = 0:0.1:1; xu = x + xu*cos(theta); xu = xu*size;
yu = 0:0.1:1; yu = y + yu*sin(theta); yu = yu*size;
p1=plot(xu, yu, 'g-');
p2=plot(xu(end), yu(end), 'r*','MarkerSize',8);
p3=plot(xu(1), yu(1), 'b.','MarkerSize',11);
drawnow;

delete(p1); delete(p2);
end