%
% Original File : DynamicWindowApproachSample.m
%
% Discription : Mobile Robot Motion Planning with Dynamic Window Approach
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

% Below is a twist of the original version only with the variable names

% -------------------------------------------------------------------------

close all; clc; clear all;
% Dynamic Window Approach Testing
global dT; dT = 0.1;  % [s] discrete time 
model.r = 0.04187;    % radius of wheel [m]
model.L = 0.18;   % wheel track [m]
model.state = [0; 0; pi/2];    % x[m], y[m], heading angle[rad]
model.ctr = [0; 0]; % v[m/s], w[rad/s] (present)

% dwa parameters
dwa.safe_dist = 0.5;   % [m], safety distance tunning variable between obstacles
dwa.eval_param = [0.1, 0.2, 0.1, 3.0]; % [heading,dist,velocity,predictDT]
dwa.lin_vel = 1.0;  % [m/s], limit linear velocity
dwa.rot_vel = deg2rad(90); % [rad/s], limit rotation velocity
dwa.lin_acc = 0.2; % [m/s^2], limit linear acceleration 
dwa.rot_acc = deg2rad(60); % [rad/s^2], limit rotation acceleration
dwa.lin_vel_intv = 0.01;    % [m/s], linear velocity interval value
dwa.rot_vel_intv = deg2rad(1);    % [rad/s], rotation velocity interval value

% obstacles
% [x(m) y(m)]
obstacles=[0 2;
          4 2;
          4 4;
          5 4;
          5 5;
          5 6;
          5 9
          8 8
          8 9
          7 9];
      
% destination
destination = [10; 10]; %[x(m),y(m)]

% plot
area = [-1, 12, -1, 12]; % [xmin xmax ymin ymax]

main(model, dwa, destination, obstacles, area);

function [] = main(model, dwa, destination, obstacles, area)
result_state = [];
result_ctr = [];
tic;    % start timer
for i=1:5000
   [u, trajectory_result] = DWA_Local(model, dwa, destination, obstacles);
   
   % update state and ctr
   model.state = F(model.state, u); % pass it through kinematic model
   model.ctr = u;
   
   % save data
   result_ctr = [result_ctr, u];
   result_state = [result_state, model.state];   % save current state

   if norm(model.state(1:2) - destination) < 0.5
        disp('Arrived at Destination!'); break;
   end
   
   %====Animation====
    hold off;
    ArrowLength = 0.5;
    quiver(model.state(1),model.state(2),ArrowLength*cos(model.state(3)),ArrowLength*sin(model.state(3)),'ok'); hold on;
    plot(result_state(1,:), result_state(2,:),'-b');hold on;
    plot(destination(1),destination(2),'*r');hold on;
    plot(obstacles(:,1),obstacles(:,2),'ok');hold on;
    if ~isempty(trajectory_result)
        for k=1:length(trajectory_result(:,1))/3    % trajectory result includes x, y, psi
            ind = 1 + (k-1)*3;
            plot(trajectory_result(ind,:), trajectory_result(ind+1,:), '-g'); hold on;    % using x and y
        end
    end
    axis(area);

    grid on;
    drawnow;
   
end

figure(2)
plot(result_ctr(1,:));
toc

end

function [u, trajectory_result] = DWA_Local(model, dwa, destination, obstacles)
Vr = calculateDynamicWindow(model.ctr, dwa);    % getting velocity min and max
[EVAL, trajectory_result] = SearchSpace(Vr, model, dwa, destination, obstacles);

if isempty(EVAL)
    disp("no path");
    u = [0;0];
    return;
end

EVAL = normalizeEVAL(EVAL);

final_eval=[];
for i = 1:length(EVAL(:,1))
    final_eval=[final_eval; dwa.eval_param(1:3)*EVAL(i,3:5)'];
end
EVAL=[EVAL final_eval];

[maxV,ind] = max(final_eval);  % choosing max value in evaluation final
u = EVAL(ind,1:2)';
end

function Vr = calculateDynamicWindow(model_ctr, dwa)
global dT;

Vs=[0 dwa.lin_vel -dwa.rot_vel dwa.rot_vel]; % 1:2 linear speed range, 3:4 angular speed range
Vd=[model_ctr(1)-dwa.lin_acc*dT model_ctr(1)+dwa.lin_acc*dT model_ctr(2)-dwa.rot_acc*dT model_ctr(2)+dwa.rot_acc*dT];

v_temp = [Vs; Vd];
Vr = [max(v_temp(:,1)) min(v_temp(:,2)) max(v_temp(:,3)) min(v_temp(:,4))]; %[vmin,vmax,ωmin,ωmax]
end

function [EVAL, TRAJ] = SearchSpace(Vr, model, dwa, destination, obstacles)
EVAL=[];
TRAJ=[];

% calculating trajectory and end point
for v = Vr(1):dwa.lin_vel_intv:Vr(2)    % linear speed
    for w = Vr(3):dwa.rot_vel_intv:Vr(4) % angular speed
        [traj, state] = control_trajectory(model.state, v, w, dwa.eval_param(4));  % last point and trajectory
        
        % evaluate for state
        heading = calcHeadings(state, destination);
        distance = calcDistance(state, obstacles, dwa.safe_dist);
        velocity = abs(v);
        
        EVAL = [EVAL; [v w heading distance velocity]];
        TRAJ = [TRAJ; traj];
    end
end
end

function [traj, model_state] = control_trajectory(model_state, v, w, predict_dT)
global dT;

ctr = [v; w];   % linear and angular velocity command
traj = model_state; % saving data for the length of predict_dT


for t = dT:dT:predict_dT
    model_state = F(model_state, ctr);  % get next state of dT
    traj = [traj, model_state];   % save moved point
end
end

function EVAL = normalizeEVAL(EVAL)
% 3, 4, 5 element refers to heading, distance, velocity

if sum(EVAL(:,3))~=0  % heading
    EVAL(:,3)=EVAL(:,3)/sum(EVAL(:,3));
end
if sum(EVAL(:,4))~=0  % distance
    EVAL(:,4)=EVAL(:,4)/sum(EVAL(:,4));
end
if sum(EVAL(:,5))~=0  % velocity
    EVAL(:,5)=EVAL(:,5)/sum(EVAL(:,5));
end

end

function heading = calcHeadings(state, destination)
% target heading angle measures the alignment of the robot with the target
% direction. It is given by 180-theta, where theta is the angle of the
% target point relative to the robot's heading direction.

diff_theta = atan2(destination(2)-state(2),destination(1)-state(1));

if diff_theta > state(3)
    target_angle = diff_theta - state(3);
else
    target_angle = state(3) - diff_theta;
end

heading = pi - target_angle;
heading = rad2deg(heading);
end

function distance = calcDistance(state, obstacles, safe_dist)
% represents the distance to the closest obstacle that intersects with the
% curvature. If no obstacle is on the curvature, this value is set t a
% large constant.

distance = 1.5;   % considering border line between obstacles 
for i = 1:length(obstacles(:,1))    % checking for each obstacle distance
    disttmp = norm(obstacles(i,:) - state(1:2)') - safe_dist;
    if distance > disttmp
        distance = disttmp;
    end
end

end

function X = F(model_state, ctr)
% Kinematic motion model: x, y, yaw
global dT;
 
f = eye(3);

b = [dT*cos(model_state(3)) 0
    dT*sin(model_state(3)) 0
    0 dT];
X = f*model_state + b*ctr;
end

%% -------------------------------------------------------------------------
%
% File : DynamicWindowApproachSample.m
%
% Discription : Mobile Robot Motion Planning with Dynamic Window Approach
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------
% DynamicWindowApproachSample();
% 
% function [] = DynamicWindowApproachSample()
%  
% close all;
% clear all;
%  
% disp('Dynamic Window Approach sample program start!!')
% 
% x=[0 0 pi/2 0 0]';%[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
% goal=[10,10];%[x(m),y(m)]
% % [x(m) y(m)]
% obstacle=[0 2;
%           4 2;
%           4 4;
%           5 4;
%           5 5;
%           5 6;
%           5 9
%           8 8
%           8 9
%           7 9];
%       
% obstacleR=0.5;
% global dt; dt=0.1;
% 
% % [m/s],[rad/s],[m/ss],[rad/ss],
% % [m/s],[rad/s]]
% Kinematic=[1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)]; % limitation of the model
% 
% % [heading,dist,velocity,predictDT]
% evalParam=[0.1,0.2,0.1,3.0];
% area=[-1 11 -1 11]; % [xmin xmax ymin ymax]
% 
% result.x=[];
% tic;
% % movcount=0;
% % Main loop
% for i=1:5000
%     [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
%     x=f(x,u);
%     
%     result.x=[result.x; x'];
%     
%     if norm(x(1:2)-goal')<0.5
%         disp('Arrive Goal!!');break;
%     end
%     
%     %====Animation====
%     hold off;
%     ArrowLength=0.5;
%     quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
%     plot(result.x(:,1),result.x(:,2),'-b');hold on;
%     plot(goal(1),goal(2),'*r');hold on;
%     plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
%     if ~isempty(traj)
%         for it=1:length(traj(:,1))/5
%             ind=1+(it-1)*5;
%             plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
%         end
%     end
%     axis(area);
%     grid on;
%     drawnow;
%     % movcount=movcount+1;
%     % mov(movcount) = getframe(gcf); 
% end
% figure(2)
% plot(result.x(:,4));
% toc
% % movie2avi(mov,'movie.avi');
% end
% 
% function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
% % Dynamic Window[vmin,vmax,ωmin,ωmax]
% Vr=CalcDynamicWindow(x,model);
% [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);
% 
% if isempty(evalDB)
%     disp('no path to goal!!');
%     u=[0;0];return;
% end
% 
% evalDB=NormalizeEval(evalDB);
% 
% feval=[];
% for id=1:length(evalDB(:,1))
%     feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
% end
% evalDB=[evalDB feval];
% 
% [maxv,ind]=max(feval);  % choosing max value of trajectory value
% u=evalDB(ind,1:2)';
% end
% 
% function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
% evalDB=[];
% trajDB=[];
% 
% for vt=Vr(1):model(5):Vr(2) % linear speed
%     for ot=Vr(3):model(6):Vr(4) % angular speed
%         [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model); % last point and trajectory
%         heading=CalcHeadingEval(xt,goal);
%         dist=CalcDistEval(xt,ob,R);
%         vel=abs(vt);
%         
%         evalDB=[evalDB;[vt ot heading dist vel]];
%         trajDB=[trajDB;traj];     
%     end
% end
% end
% 
% function EvalDB=NormalizeEval(EvalDB)
% if sum(EvalDB(:,3))~=0  % heading
%     EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
% end
% if sum(EvalDB(:,4))~=0  % dist
%     EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
% end
% if sum(EvalDB(:,5))~=0  % vel
%     EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
% end
% end
% 
% function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
% global dt;
% time=0;
% u=[vt;ot];
% traj=x;
% while time<=evaldt
%     time=time+dt;
%     x=f(x,u);
%     traj=[traj x];
% end
% end
% 
% function stopDist=CalcBreakingDist(vel,model)
% global dt;
% stopDist=0;
% while vel>0
%     stopDist=stopDist+vel*dt;
%     vel=vel-model(3)*dt;
% end
% end
% 
% function dist=CalcDistEval(x,ob,R)
% 
% dist=2;
% for io=1:length(ob(:,1))
%     disttmp=norm(ob(io,:)-x(1:2)')-R;
%     if dist>disttmp
%         dist=disttmp;
%     end
% end
% end
% 
% function heading=CalcHeadingEval(x,goal)
% theta=toDegree(x(3));
% goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));
% 
% if goalTheta>theta
%     targetTheta=goalTheta-theta;
% else
%     targetTheta=theta-goalTheta;
% end
% 
% heading=180-targetTheta;
% end
% 
% function Vr=CalcDynamicWindow(x,model)
% global dt;
% Vs=[0 model(1) -model(2) model(2)]; % 1:2 linear speed range, 3:4 angular speed range
% 
% Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];
% 
% Vtmp=[Vs;Vd];
% Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
% %[vmin,vmax,ωmin,ωmax]
% end
% 
% function x = f(x, u)
% % Motion Model, x, y, yaw, v, w
% global dt;
%  
% F = [1 0 0 0 0
%      0 1 0 0 0
%      0 0 1 0 0
%      0 0 0 0 0
%      0 0 0 0 0];
%  
% B = [dt*cos(x(3)) 0
%     dt*sin(x(3)) 0
%     0 dt
%     1 0
%     0 1];
% 
% x= F*x+B*u;
% end
% 
% function radian = toRadian(degree)
% % degree to radian
% radian = degree/180*pi;
% end
% 
% function degree = toDegree(radian)
% % radian to degree
% degree = radian/pi*180;
% end