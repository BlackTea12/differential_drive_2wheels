clc; clear all; close all;
% tracer spec
tr_L = 0.685;  % length
tr_W = 0.57;    % width
tr_H = 0.155;   % height
tr_m = 30;  % mass
tr_whD = 0.275; % wheel distance
tr_whr = 0.0255;  % wheel radius 

% robot mathematical initial state
robot.state = [0;1;pi/4];  % xr, yr, yawr
robot.ctr = [0;0];  % vr, wr
robot.ctr_ref = [0;0];

% discrete state space model
dT = 0.1;
vr = 0.8;   % m/s
Ad = [1 0 -vr*sin(robot.state(3))*dT; 0 1 vr*cos(robot.state(3))*dT; 0 0 1];
Bd = [cos(robot.state(3))*dT 0; sin(robot.state(3))*dT 0; 0 dT];
Cd = eye(3);

% % augmented model, state x = [ delta_xd(k); yd(k) ]
% [m1, n1] = size(Cd);
% [m2, n2] = size(Bd);
% Ae = eye(m2+m1, m2+m1);
% Ae(1:m2, 1:m2) = Ad;
% Ae(m2+1:m2+m1,1:m2) = Cd*Ad;
% Be = zeros(m2+m1,n2);
% Be(1:m2,:) = Bd;
% Be(m2+1:m2+m1,:) = Cd*Bd;
% Ce = zeros(m1, m2+m1);
% Ce(:, m2+1:m2+m1) = eye(m1);

% augmented model, state x = [ x(k+1); u(k) ], u length->n2, x length->n3
[m1, n1] = size(Ad);
[m2, n2] = size(Bd);
[m3, n3] = size(Cd);
% Ae = eye(n1+n2, n1+n2);
% Ae(1:m1, 1:n1) = Ad;
% Ae(1:m2, n1+1:n2+n1) = Bd;
% Be = zeros(n1+n2, n2);
% Be(1:m2, :) = Bd;
% Be(m2+1:end, :) = eye(n1+n2-m2);
% Ce = zeros(m3, n1+n2);
% Ce(:, 1:n3) = Cd;

% mpc model
Nc = 3;  % control horizon, should be equal or smaller than prediction horizon
Np = 3;  % prediction horizon
N = 3;
% h = Ce;    % initialize
% F = Ce * Ae;   % initialize

% for i = 2:Np
%     h(m3*(i-1)+1:m3*i,:) = h(m3*(i-2)+1:m3*(i-1),:) * Ae;   % C; CA; CA^2, ... -> preprocess for making \PHI
%     F(m3*(i-1)+1:m3*i,:) = F(m3*(i-2)+1:m3*(i-1),:) * Ae; % CA; CA^2; CA^3, ...
% end

% v = h*Be;   % another preprocess for making \PHI
% Phi = zeros(m3*Np, n2*Nc);
% Phi(:,1:n2) = v;
% for i = 2:Nc
%    Phi(:,n2*(i-1)+1:n2*i) = [zeros(m3*(i-1),n2); v(1:(Nc-i+1)*m3,:)];    % final \PHI 
% end
% quadratic programming
% weighting matrix gain calculation
R_vec = ones(m3*Nc,1);
Q_vec = 0.5*ones(n2*Np,1);
R = diag(R_vec,0);
Q = diag(Q_vec,0);

% augmented states : x(k+1); u(k)
% augStates = [robot.state; robot.ctr];

% reference point, k~k+Np
% W = zeros(m3*Np,1);
% 
% H = Phi'*R*Phi + Q; %hessian
% f = F*augStates;
% g_T = 2*(f-W)'*R*Phi;

% constraints calculation
% IB = lowTriDiag(1, n2*Nc);
% IL = IB;
% b = constraintb(robot.ctr, dT, Nc, n2);
% constraintA = [IB; -IB; IL; IL];

% quadratic programming
% deltaU = quadprog(H, g_T, constraintA, b, [],[],[],[],zeros(Nc*n2,1), optimoptions('quadprog','Algorithm','active-set'));

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
xr = [0:vr*cos(slope)*dT:vr*cos(slope)*dT*N_sim];  % length 101
yr = [0:vr*sin(slope)*dT:vr*sin(slope)*dT*N_sim];   % length 101
yawr = zeros(1,N_sim);  % length 100
vr = zeros(1,N_sim);
wr = zeros(1,N_sim);
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
% xr = [0:vr*dT:cos(slope)*dT*N_sim];  % length 101
% yr = zeros(1,N_sim);
% yawr = zeros(1,N_sim);  % length 100

% final form of example reference in journal "Predictive control~"
robot_ref = [xr(1:N_sim); yr(1:N_sim); yawr];
result_u = zeros(n2, N_sim);
result_states = zeros(m3, N_sim);

% initialize input robot reference
robot.ref = zeros(Np*m3,1); % [x1;y1;yaw1;x2;y2;yaw2;...]

% figure();
% plot(0,0,'.b'); hold on; grid on;
% xlim([-2 8]); ylim([-1 15]);
% xlabel('X[m]'); ylabel('Y[m]');

% test 1: simulation loop
% for t = 1: N_sim-Np
%     for k = 1:Np
%         robot.ref((k-1)*m3+1:k*m3) = robot_ref(:,t+k);
%     end
%     robot.ctr_ref = [vr(t); wr(t)];
%     
%     [du, y] = mpc_diff(robot, dT, Nc, Np, Q, R, m1, n1, m2, n2, m3, n3);
%     
%     % save data
%     result_u(:,t) = robot.ctr + du; 
%     result_states(:,t) = y;
%     
%     % data update
%     robot.ctr = result_u(:,t);
%     robot.state = result_states(:,t);
%     
%     plot(robot.state(1),robot.state(2),'r.');
%     plot(robot_ref(1,t+1),robot_ref(2,t+1),'bx');
%     drawnow;
% end
% hold off;

% plot results
% figure();
% plot(xr, yr, 'r','LineWidth',2);
% hold on; grid on;
% plot(result_states(1,1:N_sim-Np), result_states(2,1:N_sim-Np));
% hold off;

% test 2: simulation loop
for t = 1:N_sim-N
   [vtilda, wtilda] = mpc_linear(robot, dT, N, 1, 1); 
   
   ustar = [vtilda; wtilda] + robot.ctr_ref;
   robot.ctr = ustar;
   
   % ideal wmr model
   xd = robot.ctr(1) * cos(robot.state(3));
   yd = robot.ctr(1) * sin(robot.state(3));
   yawd = robot.ctr(2);
   
   % update
   robot.state = robot.state + [xd; yd; yawd] *dT;
   
end
figure();
plot([0:dT:dT*(N_sim-Np-1)],result_u(1,1:N_sim-Np),[0:dT:dT*(N_sim-Np-1)],result_u(2,1:N_sim-Np));
hold off;

function [du, y] = mpc_diff(robot, dT, Nc, Np, Q, R, m1, n1, m2, n2, m3, n3)
% discrete state space model
vr = robot.ctr_ref(1);   % m/s
Ad = [1 0 -vr*sin(robot.ref(3))*dT; 0 1 vr*cos(robot.ref(3))*dT; 0 0 1];
Bd = [cos(robot.ref(3))*dT 0; sin(robot.ref(3))*dT 0; 0 dT];
% Ad = [1 0 -vr*sin(robot.state(3))*dT; 0 1 vr*cos(robot.state(3))*dT; 0 0 1];
% Bd = [cos(robot.state(3))*dT 0; sin(robot.state(3))*dT 0; 0 dT];
Cd = eye(3);

% augmented model, state x = [ x(k+1); u(k) ], u length->n2, x length->n3
Ae = eye(n1+n2, n1+n2);
Ae(1:m1, 1:n1) = Ad;
Ae(1:m2, n1+1:n2+n1) = Bd;
Be = zeros(n1+n2, n2);
Be(1:m2, :) = Bd;
Be(m2+1:end, :) = eye(n1+n2-m2);
Ce = zeros(m3, n1+n2);
Ce(:, 1:n3) = Cd;

% mpc model
h = Ce;    % initialize
F = Ce * Ae;   % initialize

for i = 2:Np
    h(m3*(i-1)+1:m3*i,:) = h(m3*(i-2)+1:m3*(i-1),:) * Ae;   % C; CA; CA^2, ... -> preprocess for making \PHI
    F(m3*(i-1)+1:m3*i,:) = F(m3*(i-2)+1:m3*(i-1),:) * Ae; % CA; CA^2; CA^3, ...
end

v = h*Be;   % another preprocess for making \PHI
Phi = zeros(m3*Np, n2*Nc);
Phi(:,1:n2) = v;
for i = 2:Nc
   Phi(:,n2*(i-1)+1:n2*i) = [zeros(m3*(i-1),n2); v(1:(Nc-i+1)*m3,:)];    % final \PHI 
end

% augmented states : x(k+1); u(k)
augStates = [-robot.state + robot.ref(1:3,1); -robot.ctr + robot.ctr_ref];
% augStates = [robot.state; robot.ctr];

H = Phi'*R*Phi + Q; %hessian
f = F*augStates;
g_T = 2*(f-robot.ref)'*R*Phi;

% constraints calculation
IB = lowTriDiag(1, n2*Nc);
IL = IB;
b = constraintb(robot.ctr, dT, Nc, n2);
constraintA = [IB; -IB; IL; IL];

% quadratic programming
deltaU = quadprog(H, g_T, constraintA, b, [],[],[],[],zeros(Nc*n2,1), optimoptions('quadprog','Algorithm','active-set'));

% final calculation
du = deltaU(1:n2,1);    % using only first element
augStates = Ae*augStates + Be*du;
Y = Phi*deltaU + F*augStates;
y = Y(1:m3,1);
end

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
Bbar = zeros(Amn*N, Amn*N);
for i=1:N
   Bbar(Bm*(i-1)+1:Bm*i,Bn*(i-1)+1:Bn*i) = B; 
end
for i=1:N-1
    Bbar(i*Bm+1:end, Bn*(i-1)+1:Bn*i) = Abar(1:end-Amn*i,:)*B;
end

Qbar = Q*eye(Amn*N);
Rbar = R*eye(Bm*N);

% quadprog matrixes
xtilda = robot.state - robot.ref;
H = 2 * (Bbar' * Qbar * Bbar + Rbar);
f = 2*Bbar' * Qbar * Abar * xtilda;

% constraints calculation
IB = lowTriDiag(1, Bn*N);
IL = IB;
b = constraintb(robot.ctr, dT, N, Bn);
constraint = [IB; -IB; IL; IL];

% quadratic solution
ubar = quadprog(H, f, constraint, b, [],[],[],[],zeros(N*Bn,1), optimoptions('quadprog','Algorithm','active-set'));

% tilda u output
vtilda = ubar(1,1);
wtilda = ubar(2,1);
end
% function [du, y] = mpc_diff(robot, dT, Nc, Np, Q, R, m1, n1, m2, n2, m3, n3)
% % discrete state space model
% vr = 0.8;   % m/s
% % Ad = [1 0 -vr*sin(robot.ref(3))*dT; 0 1 vr*cos(robot.ref(3))*dT; 0 0 1];
% % Bd = [cos(robot.ref(3))*dT 0; sin(robot.ref(3))*dT 0; 0 dT];
% Ad = [1 0 -vr*sin(robot.state(3))*dT; 0 1 vr*cos(robot.state(3))*dT; 0 0 1];
% Bd = [cos(robot.state(3))*dT 0; sin(robot.state(3))*dT 0; 0 dT];
% Cd = eye(3);
% 
% % augmented model, state x = [ x(k+1); u(k) ], u length->n2, x length->n3
% Ae = eye(n1+n2, n1+n2);
% Ae(1:m1, 1:n1) = Ad;
% Ae(1:m2, n1+1:n2+n1) = Bd;
% Be = zeros(n1+n2, n2);
% Be(1:m2, :) = Bd;
% Be(m2+1:end, :) = eye(n1+n2-m2);
% Ce = zeros(m3, n1+n2);
% Ce(:, 1:n3) = Cd;
% 
% % mpc model
% h = Ce;    % initialize
% F = Ce * Ae;   % initialize
% 
% for i = 2:Np
%     h(m3*(i-1)+1:m3*i,:) = h(m3*(i-2)+1:m3*(i-1),:) * Ae;   % C; CA; CA^2, ... -> preprocess for making \PHI
%     F(m3*(i-1)+1:m3*i,:) = F(m3*(i-2)+1:m3*(i-1),:) * Ae; % CA; CA^2; CA^3, ...
% end
% 
% v = h*Be;   % another preprocess for making \PHI
% Phi = zeros(m3*Np, n2*Nc);
% Phi(:,1:n2) = v;
% for i = 2:Nc
%    Phi(:,n2*(i-1)+1:n2*i) = [zeros(m3*(i-1),n2); v(1:(Nc-i+1)*m3,:)];    % final \PHI 
% end
% 
% % augmented states : x(k+1); u(k)
% % augStates = [robot.state - robot.ref; robot.ctr - robot.ctr_ref];
% augStates = [robot.state; robot.ctr];
% 
% H = Phi'*R*Phi + Q; %hessian
% f = F*augStates;
% g_T = 2*(f-robot.ref)'*R*Phi;
% 
% % constraints calculation
% IB = lowTriDiag(1, n2*Nc);
% IL = IB;
% b = constraintb(robot.ctr, dT, Nc, n2);
% constraintA = [IB; -IB; IL; IL];
% 
% % quadratic programming
% deltaU = quadprog(H, g_T, constraintA, b, [],[],[],[],zeros(Nc*n2,1), optimoptions('quadprog','Algorithm','active-set'));
% 
% % final calculation
% du = deltaU(1:n2,1);    % using only first element
% augStates = Ae*augStates + Be*du;
% Y = Phi*deltaU + F*augStates;
% y = Y(1:m3,1);
% end

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