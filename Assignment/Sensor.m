function [ sensorOutput ] = Sensor(robot_x, robot_y, headingAngle, observationMatrix)
%SENSOR - Create sensor output based on observation matrix containing all
%obstacles and current heading/position of robot
%
% Syntax: [ sensorOutput ] = Sensor(robot_x, robot_y, headingAngle, observationMatrix)

% Inputs: 
%   robot_x             current x-coordinate of robot, in m
%   robot_y             current y-coordinate of robot, in m
%   headingAngle        where the robot is pointed, in radians 
%   observationMatrix   [matrix] containing all walls 
%
% Outputs: 
%   sensorOutput         distance to the nearest obstacle for both sensors (left and right) 
%       IMPORTANT NOTE: when there is no obstacle the sensor returns 1 
%
% Other m-files required: none
%
% Author: Dr. Kevin Worrall
% Last revision: 06-01-2021

%% Initialization
length = size(observationMatrix);

% Position of sensor 
xpos = robot_x + 0.1;
ypos = robot_y;


% Closest distance to nearest walls
cur_nearest_l  = 500; 
cur_nearest_r  = 500;

%% Check for obstacle
distance  = 0;
for j = 1:21
    for k = 1:100
        sensor_range = k/100;
        
        % Get Point
        theta = ((j-20)*(pi/180))+headingAngle;
        xpt = round(100*(xpos+(sensor_range*cos(theta))));
        ypt = round(100*(ypos+(sensor_range*sin(theta))));
        
        if(xpt == 0)
            xpt = 1;
        end
        
        if(ypt == 0)
            ypt = 1;
        end
        
        % True x and y
        t_ypt = ypt*0.01;
        t_xpt = xpt*0.01;
        
        % Obs Matrix
        mat_x = uint16(round((t_xpt/0.01)+500));
        mat_y = uint16(round((t_ypt/0.01)+500));

        % Get Value at point
        if (mat_x >= length(1) || mat_x <= 0)
            value = 1;
        elseif (mat_y >= length(2) || mat_y <= 0)
            value = 1;
        else
            value = observationMatrix(mat_x,mat_y);
        end

        if (value)            
            % Calculate nearest distance
            x_dis = abs(t_xpt)-abs(xpos);
            y_dis = abs(t_ypt)-abs(ypos);
            
            distance = sqrt(x_dis^2+y_dis^2);
            if (distance < cur_nearest_l)
                cur_nearest_l = distance;
            end
   
        end
    end
end

distance  = 0;
for j = 21:41
    for k = 1:100
        sensor_range = k/100;
        
        % Get Point
        theta = ((j-20)*(pi/180))+headingAngle;
        xpt = round(100*(xpos+(sensor_range*cos(theta))));
        ypt = round(100*(ypos+(sensor_range*sin(theta))));
        
        if(xpt == 0)
            xpt = 1;
        end
        
        if(ypt == 0)
            ypt = 1;
        end
        
        % True x and y
        t_ypt = ypt*0.01;
        t_xpt = xpt*0.01;
        
        % Obs Matrix
        mat_x = uint16(round((t_xpt/0.01)+500));
        mat_y = uint16(round((t_ypt/0.01)+500));

        % Get Value at point
        if (mat_x >= length(1) || mat_x <= 0)
            value = 1;
        elseif (mat_y >= length(2) || mat_y <= 0)
            value = 1;
        else
            value = observationMatrix(mat_x,mat_y);
        end
        
        if (value)
           % plot(t_ypt,t_xpt,'r*');
            
            % Calculate nearest distance
            x_dis = abs(t_xpt)-abs(xpos);
            y_dis = abs(t_ypt)-abs(ypos);
            
            distance = sqrt(x_dis^2+y_dis^2);
            if (distance < cur_nearest_r)
                cur_nearest_r = distance;
            end
        end
    end
end

if(cur_nearest_l > 100)
    cur_nearest_l = 1;
end

if(cur_nearest_r > 100)
    cur_nearest_r = 1;
end

sensorOutput = [cur_nearest_l cur_nearest_r];

end

