% function simpleTest()
close all; clc; clear all;
disp('Program started');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');          

    % Now retrieve streaming data (i.e. in a non-blocking fashion):
    t=clock;
    startTime=t(6);
    currentTime=t(6);
    
    % Initialize streaming
    sim.simxGetInt32Signal(clientID,'Keypress', sim.simx_opmode_streaming); 
    sim.simxGetStringSignal(clientID,'measuredData',sim.simx_opmode_streaming);
    
    [~, refFrame] = sim.simxGetObjectHandle(clientID, './robotFrame', sim.simx_opmode_blocking);
 	sim.simxGetObjectPosition(clientID, refFrame, -1, sim.simx_opmode_streaming);
    sim.simxGetObjectOrientation(clientID, refFrame, -1, sim.simx_opmode_streaming);

    [~, steeringLeft]  = sim.simxGetObjectHandle(clientID, './steeringLeft', sim.simx_opmode_blocking);
    [~, steeringRight] = sim.simxGetObjectHandle(clientID, './steeringRight', sim.simx_opmode_blocking);
    [~, motorLeft]     = sim.simxGetObjectHandle(clientID, './motorLeft', sim.simx_opmode_blocking);
    [~, motorRight]    = sim.simxGetObjectHandle(clientID, './motorRight', sim.simx_opmode_blocking);
    desiredSteeringAngle = 0;
    desiredWheelRotSpeed = 5;
    steeringAngleDx      = 2*pi/180;
    wheelRotSpeedDx      = 20*pi/180;
    d=0.755; %-- 2*d=distance between left and right wheels
    l=2.5772; %-- l=distance between front and read wheels

    figure(1)
    degrees = linspace(-120,120,684);
    radians = degrees.*pi/180;
    xlabel('Angle (°)'); ylabel('Distance [m]');
    grid on;
    xlim([-120,120]);
    ylim([0,10]);

    
    % Define start and goal positions
    initPos = [2, 2];
    goalPos = [-2, -2];
    ptolerance = 1;
    gtolerance = 0.1;
    state = 1;

    while (currentTime-startTime < 50)   
        
        [err, position] = sim.simxGetObjectPosition(clientID, refFrame, -1, sim.simx_opmode_buffer);
        if(err==sim.simx_return_ok)
            % fprintf('(x,y,z)=(%.1f, %.1f, %.1f)\t',position);
        end
        [err, orientation] = sim.simxGetObjectOrientation(clientID, refFrame, -1, sim.simx_opmode_buffer);
        if(err==sim.simx_return_ok)
            % fprintf('(roll,pitch,yaw)=(%.1f, %.1f, %.1f)\n',rad2deg(orientation));
        end

        pose = [position(1:2) orientation(3)];

        [err,signal]=sim.simxGetStringSignal(clientID,'measuredData',sim.simx_opmode_buffer);
        if (err==sim.simx_return_ok)
            % Data produced by the child script was retrieved!
            unpackedsignal = sim.simxUnpackFloats(signal);
            %------------------- Debug -------------------
            % lidar = unpackedsignal(unpackedsignal~=2);  % removes measurements above 2mts
            % radians2 = radians(unpackedsignal~=2);      % removes measurements above 2mts
            % disp(unpackedsignal)
            % p = polarscatter(radians2,lidar);
            % [x,y] = pol2cart(radians2,lidar);

            [dfront, dleft_front, dleft_back, dright_front, dright_back, r_ang, l_ang] = getLidarMeasurements(unpackedsignal,radians);
            
            [pointing,direction,angle_diff] = point2goal(pose,initPos,goalPos,ptolerance);
            side = lineSide(pose,initPos,goalPos);
            hasReachGoal = goalReached(pose,goalPos,gtolerance);

            if(state==1)
                % Check if it is pointing to goal
                [pointing2goal,direction,angle_diff] = point2goal(pose,initPos,goalPos,ptolerance);
                if(pointing2goal)
                    state=2;
                else
                    desiredSteeringAngle = go2goal(pointing2goal,direction,angle_diff);
                end
            elseif(state==2)
                % Check if it is pointing to goal
                [pointing2goal,direction,angle_diff] = point2goal(pose,initPos,goalPos,ptolerance);
                % Check if goal has been reached
                hasReachGoal = goalReached(pose,goalPos,gtolerance);
                % Check if there is object in front
                objectInFront = dfront < 1;
                if(hasReachGoal)
                    state=6;
                elseif(~pointing2goal)
                    state=1;
                elseif(objectInFront)
                    direction = 1; % turn right
                    state=3;
                end
            %------------- Wall Follower Routine -------------
            % TODO_1: detect when to get out of wall routine by using
            %       lineSide function
            % TODO_2: improve routine that drives robot towards goal
            elseif(state==3)    
                if(dleft_front<1 && dleft_back>1)
                    state=4;
                    desiredSteeringAngle = -45*pi/180;
                end
            elseif(state==4)
                if(dleft_front<1 && dleft_back<1)
                    gamma = 1*(0.5-0.5*(dleft_front+dleft_back));
                    desiredSteeringAngle = l_ang*pi/180 - gamma;
                elseif(dleft_front>1 && dleft_back<1)
                    state=5;
                end
            elseif(state==5)
                if(dleft_front>1 && dleft_back<1) 
                    desiredSteeringAngle = 45*pi/180;
                else
                    state=4;
                end
                fprintf('state:%i\tlfrnt_d: %.2f\tl_ang: %.2f\tgamma: %.2f\tdes_ang: %.2f\n',state,dleft_front,l_ang,gamma,desiredSteeringAngle)
            elseif(state==6)
                disp('state 6, stop motors')
            end
            
            % fprintf('pointing: %i\tdirection: %i\tangle: %.2f\tside: %i\tgoal: %i\n',pointing,direction,angle_diff,side,hasReachGoal);
            
        end

        % Get Keyboard input
        [KeyboardReturnCode, signalValue] = sim.simxGetInt32Signal(clientID,'Keypress', sim.simx_opmode_buffer);

        if (KeyboardReturnCode==sim.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code                
            if (signalValue==2007)
                %-- up key
                desiredWheelRotSpeed = desiredWheelRotSpeed + wheelRotSpeedDx;
                % fprintf('Up key,\t RotSpeed = %d\n',desiredWheelRotSpeed*180/pi);
            end
            if (signalValue==2008)
                %-- down key
                desiredWheelRotSpeed = desiredWheelRotSpeed - wheelRotSpeedDx;
                % fprintf('Down key,\t RotSpeed = %d\n',desiredWheelRotSpeed*180/pi);
            end
            if (signalValue==2009)
                %-- left key
                desiredSteeringAngle = desiredSteeringAngle + steeringAngleDx;
                if (desiredSteeringAngle>45*pi/180) 
                    desiredSteeringAngle=45*pi/180;
                end
                % fprintf('Left key,\t SteerAngle = %d\n',desiredSteeringAngle*180/pi);
            end
            if (signalValue==2010)
                %-- right key
                desiredSteeringAngle = desiredSteeringAngle - steeringAngleDx;
                if (desiredSteeringAngle<-45*pi/180)
                    desiredSteeringAngle=-45*pi/180;
                end
                % fprintf('Right key,\t SteerAngle = %d\n',desiredSteeringAngle*180/pi);
            end
        end
    
        %-- We handle the front left and right wheel steerings (Ackermann steering):
        steeringAngleLeft = atan(l/(-d+l/tan(desiredSteeringAngle)));
        steeringAngleRight= atan(l/(d+l/tan(desiredSteeringAngle)));
        [~] = sim.simxSetJointTargetPosition(clientID, steeringLeft, steeringAngleLeft, sim.simx_opmode_streaming);
        [~] = sim.simxSetJointTargetPosition(clientID, steeringRight, steeringAngleRight, sim.simx_opmode_streaming);

        %-- We take care of setting the desired wheel rotation speed:
        [~] = sim.simxSetJointTargetVelocity(clientID, motorLeft , desiredWheelRotSpeed , sim.simx_opmode_streaming);
        [~] = sim.simxSetJointTargetVelocity(clientID, motorRight , desiredWheelRotSpeed , sim.simx_opmode_streaming);

        pause(0.01) 

        t=clock;
        currentTime=t(6);
    end

    % Stop motors
    [~] = sim.simxSetJointTargetVelocity(clientID, motorLeft , 0 , sim.simx_opmode_streaming);
    [~] = sim.simxSetJointTargetVelocity(clientID, motorRight , 0 , sim.simx_opmode_streaming);
        
    % Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);

    % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID);

    % Now close the connection to CoppeliaSim:    
    sim.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended');
% end

%% Functions not implemented in main code
function Bug0Algorithm(start, goal, obstacles)
    % Initialize robot position
    robotPos = start;
    goalReached = false;
    
    % Plot the grid and obstacles
    figure;
    hold on;
    axis([0, 20, 0, 20]);
    plot(start(1), start(2), 'ro', 'MarkerSize', 10); % Start position
    plot(goal(1), goal(2), 'go', 'MarkerSize', 10); % Goal position
    plot(obstacles(:,1), obstacles(:,2), 'kx', 'MarkerSize', 10); % Obstacles
    plot(robotPos(1), robotPos(2), 'bo'); % Initial robot position
    
    % Main loop
    while ~goalReached
        % Move towards the goal until an obstacle is encountered
        while ~goalReached && ~hasEncounteredObstacle(robotPos, obstacles)
            % Move towards the goal
            robotPos = moveTowardsGoal(robotPos, goal);
            plot(robotPos(1), robotPos(2), 'bo'); % Plot the robot's movement
            
            % Check if the goal is reached
            goalReached = hasReachedGoal(robotPos, goal);
        end
        
        % If an obstacle is encountered, move around it
        if ~goalReached
            robotPos = moveAroundObstacle(robotPos, goal, obstacles);
        end
    end
    
    disp('Goal reached!');
end

function reached = hasReachedGoal(currentPos, goalPos)
    reached = norm(currentPos - goalPos) < 1; % Adjust tolerance as needed
end

function newPos = moveTowardsGoal(currentPos, goalPos)
    direction = goalPos - currentPos;
    direction = direction / norm(direction); % Normalize the direction vector
    stepSize = 0.5; % Adjust step size as needed
    newPos = currentPos + stepSize * direction;
end

function encountered = hasEncounteredObstacle(robotPos, obstacles)
    thresholdDistance = 1.5; % Adjust threshold as needed
    distances = vecnorm(obstacles - robotPos, 2, 2);
    encountered = any(distances < thresholdDistance);
end

function newPos = moveAroundObstacle(currentPos, goal, obstacles)
    % Find the nearest obstacle
    nearestObstacle = obstacles(find(vecnorm(obstacles - currentPos, 2, 2) < 1.5), :);
    
    % Move around the obstacle
    obstaclePos = nearestObstacle;
    while true
        direction = goal - obstaclePos;
        direction = direction / norm(direction); % Normalize the direction vector
        stepSize = 0.2; % Adjust step size as needed
        obstaclePos = obstaclePos + stepSize * direction;
        plot(obstaclePos(1), obstaclePos(2), 'ro'); % Plot the obstacle's movement
        
        % Check if the path around the obstacle reaches the goal
        if norm(obstaclePos - goal) < 1
            newPos = obstaclePos;
            break;
        end
    end
    
    % Move towards the goal from the obstacle's position
    newPos = moveTowardsGoal(obstaclePos, goal);
end

