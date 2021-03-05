function [booleanAtCheckpoint, newHeadingAngle] = ComputeHeadingAngle(currentLocation, checkpoint, tolerance)
%COMPUTEHEADINGANGLE - Compute the heading angle to the next checkpoint
%
% Syntax: [booleanAtCheckpoint, newHeadingAngle] = ComputeHeadingAngle(currentLocation, checkpoint, tolerance)

% Inputs: 
%   currentLocation     x- and y-coordinates of position, you can use [state(19), state(20)];
%   checkpoint          checkpoint to aim for
%   tolerance           tolerance for acceptance radius around checkpoint
%
% Outputs: 
%   booleanAtCheckpoint 1: at checkpoint, 0: not at checkpoint
%   newHeadingAngle     in radians, heading angle required to go to checkpoint
%
% Other m-files required: none
%
% Author: Merel Vergaaij
% Last revision: 06-01-2021


% Check to see if within tolerance 
deltaPosition = checkpoint - currentLocation;
distanceFromCheckpoint = norm(deltaPosition);
%disp(deltaPosition);
booleanAtCheckpoint = (distanceFromCheckpoint < tolerance);

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
