function side = lineSide(pose,initPos,goalPos)
    % Detects when robot is on one side of the line between initPos and
    % goalPos.

    xDiff1=initPos(1)-goalPos(1);
    yDiff1=goalPos(2)-initPos(2);
    xDiff2=initPos(1)-pose(1);
    yDiff2=initPos(2)-pose(2);
    dGoalIni=sqrt(xDiff1^2+yDiff1^2);
    d=(xDiff2*yDiff1+yDiff2*xDiff1)/dGoalIni;
    if(d>0)
        side=1;
    else
        side=-1;
    end
end

