function [] = DrawRobot(robot_radius, robot_x, robot_y, headingAngle, colour)
%DRAWROBOT - Draws a representation of the robot and its orientation
%
% Syntax: [] = DrawRobot(r_robot, centre_x, centre_y, psi_rad, colour)

% Inputs: 
%   robot_radius    radius of robot
%   robot_x         x-coordinate of centre of robot 
%   robot_y         y-coordinate of centre of robot
%   headingAngle    current heading angle, in radians
%   colour          RGB or matlab-color (e.g. 'r', 'm');
%
% Outputs: none (plot to current figure)
%
% Other m-files required: none
%
% Author: Dr. Kevin Worrall
% Last revision: 06-01-2021

%% Generate Circle
theta    = linspace(0, 2*pi, 360);
circle_x = robot_radius * cos(theta);
circle_y = robot_radius * sin(theta);

%% Calculate end points of the line
% Line is 0.02m larger than the robot
lineEnd_x = cos(headingAngle) * (robot_radius + 0.02);
lineEnd_y = sin(headingAngle) * (robot_radius + 0.02);

%% Draw robot
% Draw circle around current location of robot
plot((circle_x + robot_x), (circle_y + robot_y), colour);

% Draw line from center of robot in the direction of the current heading
line([robot_x, (lineEnd_y + robot_x)], [robot_y, (lineEnd_x + robot_y)], 'color', colour);

end