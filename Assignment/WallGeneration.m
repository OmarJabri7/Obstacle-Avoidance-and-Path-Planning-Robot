function [ wall, obstacleMatrix_new ] = WallGeneration(start_y, end_y, start_x, end_x, direction, obstacleMatrix)
%WALLGENERATION - Construct a wall the robot cannot pass through
%
% Syntax: [ wall, obstacleMatrix_new  ] = WallGeneration(start_y, end_y, start_x, end_x, direction, obstacleMatrix)
%
% Inputs: 
%   start_y              y-coordinate of start
%   end_y                y-coordinate of end 
%   start_x              x-coordinate of start
%   end_x                x-coordinate of end
%   direction            'h' or 'v' (horizontal or vertical, respectively)
%   obstacleMatrix    obstacle matrix before adding this wall
%
% Outputs: 
%   wall                 matrix containing the wall coordinates
%   obstacleMatrix_new   obstacle matrix after adding this wall
%
% Other m-files required: none
%
% Author: Dr. Kevin Worrall
% Last revision: 06-01-2021

%% Inputs and pre-processing
length_y = abs(start_y) + abs(end_y);
length_x = abs(start_x) + abs(end_x);

stepSize_canvas = 0.01;

obstacleMatrix_new = obstacleMatrix;
canvasSize_horizontal = size(obstacleMatrix,1) * stepSize_canvas;
canvasSize_vertical   = size(obstacleMatrix,2) * stepSize_canvas;

%% Generate wall
% Horizontal wall
if direction == 'h'
    wall(1,:) = [start_y, start_x];
    for step = 2:length_y/stepSize_canvas
        wall(step,:) = [start_y + (step * stepSize_canvas), start_x];
    end
end

% Vertical wall
if direction == 'v'
    wall(1,:) = [start_y, start_x];
    for step = 1:length_x/stepSize_canvas
        wall(step,:) = [start_y, start_x + (step * stepSize_canvas)];
    end
end

for wallElement = 1:length(wall)    
    position_x = ceil( ( wall(wallElement,1) / stepSize_canvas ) + ( (canvasSize_horizontal/2) / stepSize_canvas ) );
    position_y = ceil( ( wall(wallElement,2) / stepSize_canvas ) + ( (canvasSize_vertical  /2) / stepSize_canvas ) );
    
    obstacleMatrix_new(position_y, position_x) = 1;
end

end

