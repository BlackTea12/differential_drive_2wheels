clc; close all; clear all;
%%
c=0; l=0.05; L=0.05; 
r=0.02; vx=0.15;
k=0.1; beta=2*l*(1-sign(vx)); ki=0.25;
L1= 500; L2=-5*10^4*sign(vx);
%%
% initialize variables
start = [0 0 0]; goal = [0 0.5 0];
state.current = [0 0 0]; state.past = [0 0 0]; 
r = 0.06; L = 0.6; dt = 1/15;

% default fixed figure
figure();
plot(0,0,'xb'); grid on; hold on;
xlim([-1 1]); ylim([-1 1]);

% test short simulation
for i=1:150
%     alpha = goal(3) - pose(3);
%     beta = sqrt((goal(1) - pose(1))^2 + (goal(2) - pose(2))^2);
%     [v, w] = DifferentialDriveControl(alpha, beta);
%     [vr, vl] = DifferentialWheelControl(v, w, r, L);
%     [xdot, ydot, thetadot] = DifferentialDriveModel(vr, vl, w, r, L);
%     pose(1) = pose(1) + xdot * dt;
%     pose(2) = pose(2) + ydot * dt;
%     pose(3) = pose(3) + thetadot * dt;
%     plot(pose(1),pose(2),'b.');
%     drawnow;
    %Vec2D(pose(1),pose(2),pose(3),abs(v));
    
    %alpha = -(state.current(3) - state.past(3))/dt + ;
    beta = sqrt((goal(1) - state.current(1))^2 + (goal(2) - state.current(2))^2);
    [v, w] = DifferentialDriveControl(alpha, beta);
    [vr, vl] = DifferentialWheelControl(v, w, r, L);
    statedot = inverseDifferentialKinematics(vr, vl, state.current(3), r, L);
    
    % state update
    state.past = state.current;
    state.current = state.past + dt * [statedot.x statedot.y statedot.theta];
    
    plot(state.current(1), state.current(2), 'b.');
    drawnow;
    disp(rad2deg(state.current(3)));
end

% input: right/left wheel velocity, heading angle, wheel radius, wheel
% distance
% output: state dot
function statedot = inverseDifferentialKinematics(vr, vl, theta, R, L)
statedot.x = R/2 * (vr+vl)*cos(theta);
statedot.y = R/2 * (vr+vl)*sin(theta);
statedot.theta = R/L * (vr-vl);
end

% input: velocity, angular velocity, wheel radius, wheel distance
% output: right wheel velocity, left wheel velocity
function [vr, vl] = DifferentialWheelControl(v, w, R, L)
vr = (2*v + w*L)/(2*R);
vl = (2*v - w*L)/(2*R);
end

function [v, w] = DifferentialDriveControl(alpha, beta)
% tuning value
Kp = 0.5;
Ka = 0.5;
v = Kp*beta*cos(alpha);
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