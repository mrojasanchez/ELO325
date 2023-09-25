clear; close all;

%%% Control variables
n = 300;                            % number of points
deltatime = 0.05;                   

%%% Draw path (circle)
r = 2;                              % Radius of path [mts]
c = [0 r];                          % Center of path

theta = linspace(0,2*pi,n);         % running variable [radians]
x_goal = c(1) + r*sin(theta);
y_goal = c(2) + r*cos(theta-pi);
z_goal = ones(size(y_goal))*0.1;

%%% Compute Robot Motion
robot_xy = [x_goal; y_goal];        % Position data
robot_yaw = theta;                  % Orientation data (XYZ Euler angles)
robot_pose = [robot_xy; robot_yaw];

A = @(th) [cos(th) 0;
           sin(th) 0;
           0       1];

timesteps = linspace(0,n*deltatime,n-1);
for i=1:length(timesteps)
    x(:,i) = A(theta(i))\(robot_pose(:,i+1)-robot_pose(:,i)).*(1/deltatime); 
end

% Angular velocities of wheels
r_wheel = 0.15;                     % Radius of wheel
L_car = 0.3;                        % Distance between wheels

Q = [r_wheel/2          r_wheel/2;
             0                  0;
    -r_wheel/L_car  r_wheel/L_car];

for i=1:length(timesteps)
    v_root = x(1,i);
    w_root = x(2,i);
    w_wheel(:,i) = Q\[v_root; 0; w_root];
end

%%% Draw animation
y_goal(end) = NaN;
xyz_goal = [x_goal;y_goal;z_goal];

robot_rpy = [zeros([2 length(robot_yaw)]); robot_yaw];
  
params.plot_path_num = false;       % Enable/disable path numbers

draw3D(xyz_goal', robot_rpy', timesteps, xyz_goal, params);

%%% Plot Robot states
figure;
ax1 = subplot(4,1,1);
plot(timesteps,x(1,:))
title("Root velocity")
ax1.YAxis.TickLabelFormat = '%.2f';

ax2 = subplot(4,1,2);
plot(timesteps,x(2,:))
title("Root angular vel")
ax2.YAxis.TickLabelFormat = '%.2f';

ax3 = subplot(4,1,3);
plot(timesteps,w_wheel(1,:))
title("Wheel angular vel left")
ax3.YAxis.TickLabelFormat = '%.2f';

ax4 = subplot(4,1,4);
plot(timesteps,w_wheel(2,:))
title("Wheel angular vel rigth")
ax4.YAxis.TickLabelFormat = '%.2f';
