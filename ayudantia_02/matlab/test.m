clear; close all;

%%% Compute Robot Motion
timestep = (0:0.001:1)';                                        % Time data
robot_xyz = [0.5*sin(2*pi*timestep), 0*timestep, 0*timestep];   % Position data
robot_rpy = [0*timestep, 0*timestep, 0*timestep];               % Orientation data (XYZ Euler angles)

%%% Draw path
r = 2;                              % Radius
c = [0 r];                          % Center
n = 30;                             % number of points

theta = linspace(0,2*pi,n);         % running variable
x_goal = c(1) + r*sin(theta);
y_goal = c(2) + r*cos(theta-pi);
y_goal(end) = NaN;
z_goal = ones(size(y_goal))*0.1;

xyz_goal = [x_goal;y_goal;z_goal];

%%% 
draw3D(robot_xyz, robot_rpy, timestep, xyz_goal);


