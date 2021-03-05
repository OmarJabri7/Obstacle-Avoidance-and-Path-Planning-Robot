function [turn_angle] = limitAngle(currentHeadingAngle,newHeadingAngle)
if(newHeadingAngle >= currentHeadingAngle)
    turn_angle = newHeadingAngle - currentHeadingAngle;
    if(turn_angle > pi)
        turn_angle = turn_angle - 2*pi;
    end
else
    turn_angle = newHeadingAngle - currentHeadingAngle;
    if(turn_angle < -pi)
        turn_angle = turn_angle + 2*pi;
    end
end
end

