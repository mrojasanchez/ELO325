function hasReachGoal = goalReached(pose,goalPos,tolerance)
    dGoal = sqrt((goalPos(1)-pose(1))^2 + (goalPos(2)-pose(2))^2);
    hasReachGoal = dGoal<tolerance;
end

