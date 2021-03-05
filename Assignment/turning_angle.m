function [newAngle] = turning_angle(currentAngle,headingAngle)
%TURNING_ANGLE Summary of this function goes here
%   Detailed explanation goes here
newAngle = headingAngle * (180/pi) - currentAngle * (180/pi);
% newAngle = headingAngle - currentAngle;
end