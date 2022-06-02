clc; clear all; close all;

% tracer spec
tr_L = 0.685;  % length
tr_W = 0.57;    % width
tr_H = 0.155;   % height
tr_m = 30;  % mass
tr_whD = 0.275; % wheel distance
tr_whr = 0.0255;  % wheel radius 

% robot mathematical initial state
robot.state = [0;-0.5;0];  % xr, yr, yawr
robot.ref = [0;0;0];
robot.ctr = [0;0];  % vr, wr
robot.ctr_ref = [0;0];

% variables
dT = 0.1;

% prediction and control horizon as N
N = 10;

% reference 1
N_sim = 100;
% xr = sin(2*[0:pi/N_sim:pi]);  % length 101
% yr = 5*sin([pi/2:pi/N_sim:3*pi/2]);   % length 101
% yawr = zeros(1,N_sim);  % length 100
% for i=1:N_sim
%     yawr(i) = atan2(dT*(yr(i+1)-yr(i)), dT*(xr(i+1)-xr(i)));
% end

% reference 2
slope = pi/4;   % 1
% xr = [0:vr*cos(slope)*dT:vr*cos(slope)*dT*N_sim];  % length 101
% yr = [0:vr*sin(slope)*dT:vr*sin(slope)*dT*N_sim];   % length 101
% yawr = zeros(1,N_sim);  % length 100
% vr = zeros(1,N_sim);
% wr = zeros(1,N_sim);
% for i=1:N_sim
%    dxr = (xr(i+1) - xr(i))/dT;
%    dyr = (yr(i+1) - yr(i))/dT;
%    
%    if i == 1
%     ddxr = ((xr(i+1) - xr(i))/dT - xr(i)/dT) / dT;
%     ddyr = ((yr(i+1) - yr(i))/dT - yr(i)/dT) / dT;
%    else
%     ddxr = ((xr(i+1) - xr(i))/dT - (xr(i) - xr(i-1))/dT) / dT;
%     ddyr = ((yr(i+1) - yr(i))/dT - (yr(i) -yr(i-1))/dT) / dT;
%    end
%    
%    vr(i) = sqrt(dxr^2 + dyr^2);
%    yawr(i) = atan2(dyr, dxr);
%    wr(i) = (dxr * ddyr - dyr*ddxr)/ vr(i);
% end


% reference 3
xr = [0:0.5*dT:0.5*dT*N_sim];  % length 101
yr = [0:0.5*dT:0.5*dT*N_sim];
yawr = slope*ones(1,N_sim);  % length 100

% final form of example reference in journal "Predictive control~"
% robot_ref = [xr(1:N_sim); yr(1:N_sim); yawr];

% data savings and plots
result_u = zeros(length(robot.ctr), N_sim);
result_states = zeros(length(robot.state), N_sim);

% simulation loop
for t = 1:N_sim
   % ref
   robot.ref = [xr(t); yr(t); yawr(t)];
   robot.ctr_ref = [0.3; 0];
   
   % quad prog
   [vtilda, wtilda] = mpc_linear(robot, dT, N, 0.5, 0.5); 
   
   ustar = [vtilda; wtilda] + robot.ctr_ref;
   robot.ctr = ustar + robot.ctr_ref;
   
   % ideal wmr model
   xd = robot.ctr(1) * cos(robot.state(3));
   yd = robot.ctr(1) * sin(robot.state(3));
   yawd = robot.ctr(2);
   
   % update
   robot.state = robot.state + [xd; yd; yawd] *dT;
   result_u(:,t) = robot.ctr;
   result_states(:,t) = robot.state;
   
end

figure();
plot([dT:dT:dT*N_sim],result_u(1,1:N_sim)); hold on;
plot([dT:dT:dT*N_sim],result_u(2,1:N_sim))
grid on; hold off;

figure();
plot(result_states(1,1:N_sim), result_states(2,1:N_sim)); hold on;
plot(xr, yr,'--');
title('trajectory'); xlabel('x[m]'); ylabel('y[m]');
legend('mpc', 'reference');
grid on; hold off;

figure();
plot([dT:dT:dT*N_sim], result_states(3,1:N_sim)); hold on;
plot([dT:dT:dT*N_sim], yawr,'--');
title('heading angle');
legend('mpc', 'reference');
grid on; hold off;

function [vtilda, wtilda] = mpc_linear(robot, dT, N, Q, R)
A = [1 0 -robot.ctr_ref(1)*sin(robot.ref(3))*dT; 0 1 robot.ctr_ref(1)*cos(robot.ref(3))*dT; 0 0 1];
B = [cos(robot.ref(3))*dT 0; sin(robot.ref(3))*dT 0; 0 dT];

Amn = length(A);
Abar = zeros(Amn * N, Amn);
Abar(1:Amn,:) = A;
for i=2:N
    Abar ((i-1)*Amn+1:i*Amn,:) = A * Abar((i-2)*Amn+1:(i-1)*Amn,:);
end

Bm = size(B,1); Bn = size(B,2);
Bbar = zeros(Bm*N, Bn*N);
for i=1:N
   Bbar(Bm*(i-1)+1:Bm*i,Bn*(i-1)+1:Bn*i) = B; 
end
for i=1:N-1
    Bbar(i*Bm+1:end, Bn*(i-1)+1:Bn*i) = Abar(1:end-Amn*i,:)*B;
end

Qbar = Q*eye(Amn*N);
Rbar = R*eye(Bn*N);

% quadprog matrixes
xtilda = robot.state - robot.ref;
H = 2 * (Bbar' * Qbar * Bbar + Rbar); hessian = (H+H')/2;
f = 2*Bbar' * Qbar * Abar * xtilda;

% constraints calculation
IB = lowTriDiag(1, Bn*N);
IL = IB;
b = constraintb(robot.ctr, dT, N, Bn);
constraint = [IB; -IB; IL; IL];

% quadratic solution
ubar = quadprog(hessian, f, constraint, b, [],[],[],[],zeros(N*Bn,1),optimoptions('quadprog','Algorithm','active-set'));

% tilda u output
vtilda = ubar(1,1);
wtilda = ubar(2,1);
end

function lowTri = lowTriDiag(val, sizeM)
% numbers of val to be put in
% sigma sizeM
n = sizeM * (sizeM+1) / 2;
lowTri = zeros(sizeM);

x = 1; y = 1;
for i = 1:n
    lowTri(x,y) = val;
    x = x+1;
    if x > sizeM
        y = y+1;
        x = y;
    end
end
end

function b = constraintb(u, dT, Nc, u_size)
dumax = 4*pi*dT;
dumin = -4*pi*dT;
umax = pi;
umin = -pi;
% ymax = [1;
%         1];
% ymin = [-1;
%         -1];

b1 = zeros(Nc*u_size,1); b2 = zeros(Nc*u_size,1);
b3 = zeros(Nc*u_size,1); b4 = zeros(Nc*u_size,1);
% b5 = zeros(Nu,1); b6 = zeros(Nu,1);

for i = 1:Nc
    b1((i-1)*u_size+1:i*u_size) = dumax;
    b2((i-1)*u_size+1:i*u_size) = -dumin;
    b3((i-1)*u_size+1:i*u_size) = umax - u;
    b4((i-1)*u_size+1:i*u_size) = -(umin - u);
%     b5(u_size*i-1:u_size*i,1) = ymax - f(u_size*i-1:u_size*i,1);
%     b6(u_size*i-1:u_size*i,1) = -(ymin - f(u_size*i-1:u_size*i,1));
end

% b = [b1;b2;b3;b4;b5;b6];
b = [b1;b2;b3;b4];
end