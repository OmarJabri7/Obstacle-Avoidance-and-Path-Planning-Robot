function [left_wheel,right_wheel] = pure_pursuit(lookahead,position,robotAngle)
%PURE_PURSUIT Summary of this function goes here
%   Detailed explanation goes here
robot_radius = 2;
side = sign(sin(robotAngle) * (lookahead(1) - position(1)) - cos(robotAngle) * (lookahead(2) - position(2)));
a = - tan(robotAngle);
c = tan(robotAngle) * position(1) - position(2);
x = abs(a*lookahead(1) + lookahead(2) + c)/sqrt(a^2 + 1);
curvature = side * (2 * x / (double(1)^2));
left_wheel = (2 - curvature * robot_radius)/2;
right_wheel = (2 + curvature * robot_radius)/2;
end