function [pointing,direction,angle_diff] = point2goal(pose,initPos,goalPos,tolerance)
    % Function to detect when the robot is pointing in the direccion of the
    % goal and how much.
    % Input:
    %   pose: [x,y,z] coordinate of the robot.
    %   initPos: [x,y,z] coordinate of the initial position.
    %   goalPos: [x,y,z] coordinate of the goal position.
    %   tolerance: float number used for dead zone where the robot is
    %               considered pointing in the direction of the goal.
    % Output:
    %   pointing: boolean, indicates if is pointing towards goal.
    %   direction: +1 if needs to rotate counter clockwise, -1 otherwise.
    %   angle_diff: angle difference between goal-init vector and robot vector.
    
    % angle_diff = atan2(goalPos(2)-initPos(2),goalPos(1)-initPos(1)) - pose(3);
    angle_diff = rad2deg(atan2(goalPos(2)-initPos(2),goalPos(1)-initPos(1))) - rad2deg(pose(3));

    %----------- Debug -----------
    % if(pose(3)<0)
    %     pose(3) = pose(3)+2*pi;
    % end
    % pose(3) = rad2deg(pose(3));
    % 
    % lineAng = atan2(goalPos(2)-initPos(2),goalPos(1)-initPos(1));
    % if(lineAng<0)
    %     lineAng = lineAng+2*pi;
    % end
    % lineAng = rad2deg(lineAng);
    % 
    % angle_diff = lineAng - pose(3);
    % 
    % fprintf('lineAng: %.2f\tyaw: %.2f\tang_diff: %.2f\n',lineAng, pose(3),angle_diff);

    if(abs(angle_diff)<tolerance)
        pointing=true;
    else
        pointing=false;
    end
    if(angle_diff>0)
        direction=1;
    else
        direction=-1;
    end
end

