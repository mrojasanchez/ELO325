function desiredSteeringAngle = go2goal(pointing,direction,angle_diff)
    if(~pointing)
        if(abs(angle_diff)>45)
            desiredSteeringAngle = 45*direction;
        else
            desiredSteeringAngle = angle_diff*pi/180;
        end
    else
        desiredSteeringAngle = 0;
    end
end

