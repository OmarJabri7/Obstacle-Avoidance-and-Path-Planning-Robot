function [reward] = calculateReward(currentLocation,map,checkpoint,tolerance)
%CALCULATEREWARD Summary of this function goes here
%   Detailed explanation goes here
deltaPosition = checkpoint - currentLocation;
distanceFromCheckpoint = norm(deltaPosition);
if(map(currentLocation(1),currentLocation(2)) == 1)
    reward = -10;
elseif (distanceFromCheckpoint < tolerance)
    reward = +10;
else 
    reward = 0;
end
end

