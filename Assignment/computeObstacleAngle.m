function [newHeadingAngle] = computeObstacleAngle(deltaPosition)
% Check to see if within tolerance 

% Calculate new heading
calculatedHeadingAngle = atan2( deltaPosition(2), deltaPosition(1) );

if isnan(calculatedHeadingAngle)
    if deltaPosition(2) >= 0
        newHeadingAngle = pi/2;
    else
        newHeadingAngle = -pi/2;
    end
else
    newHeadingAngle = calculatedHeadingAngle;
end

end


