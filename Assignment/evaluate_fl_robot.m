%RUNMODEL - Main code to run robot model for ENG5009 class
%
%   NOTE: Students should only change the code between *---* markers
%
% Syntax: RunModel
%
% Inputs: none
% Outputs: none
% 
% Other m-files required: 
%   Sensor.m
%   WallGeneration.m
%   DynamicalModel.m
%   DrawRobot.m
%
% Author: Dr. Kevin Worrall
% Last revision: 06-01-2021

%% Preamble
close all;
clear all;
clc;
%% Simulation setup
% Time
simulationTime_total = 20;           % in seconds *------* YOU CAN CHANGE THIS
stepSize_time = 0.05;               % in seconds 

% Initial states and controls
voltage_left  = 6;                  % in volts *------* YOU CAN CHANGE THIS
voltage_right = 6;                  % in volts *------* YOU CAN CHANGE THIS

state_initial = zeros(1,24);        % VARIABLES OF IMPORTANCE:
                                    % state_initial(13): forward velocity,    v, m/s
                                    % state_initial(18): rotational velocity, r, rad/s
                                    % state_initial(19): current x-position,  x, m
                                    % state_initial(20): current y-position,  y, m
                                    % state_initial(24): heading angle,       psi, rad
                                    
% Environment
canvasSize_horizontal = 10;
canvasSize_vertical   = 10;
stepSize_canvas       = 0.01;

% *------------------------------------------*
%  YOU CAN ADD MORE SETUP HERE 
%  (e.g. setup of controller or checkpoints)
% *------------------------------------------*

%% Create Environment
obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);

% Generate walls
% --> the variable "obstacleMatrix" is updated for each added wall
[wall_1, obstacleMatrix] = WallGeneration( -1,  1, 1.2, 1.2, 'h', obstacleMatrix);
[wall_2, obstacleMatrix] = WallGeneration( -3,  -1, 1.2, 1.2, 'h', obstacleMatrix);
[wall_3, obstacleMatrix] = WallGeneration( -1,  2, -2.2, -2.2, 'h', obstacleMatrix); 
[wall_4, obstacleMatrix] = WallGeneration( -3, -3,  -2,   2, 'v', obstacleMatrix);
[wall_5, obstacleMatrix] = WallGeneration(3,3,-2,2,'v',obstacleMatrix);
[wall_6, obstacleMatrix] = WallGeneration( -1.2, -1.2,  -2,   2, 'v', obstacleMatrix);

% *---------------------------*
%  YOU CAN ADD MORE WALLS HERE
% *---------------------------*

%% Main simulation
% Initialize simulation 
timeSteps_total = simulationTime_total/stepSize_time;
state = state_initial;
time = 0;
robot = readfis("robot.fis");
%% 
% Run simulation
for timeStep = 1:timeSteps_total
    state(1,20) = 4;
    state(1,19) = 0;
    state(1,24) = -pi/3;
    sensorIn = Sensor(state(timeStep,19), state(timeStep,20),state(timeStep,24), obstacleMatrix);
    disp(sensorIn);
    voltagesOut = round(evalfis(robot,sensorIn));
    disp(voltagesOut);
    % Assign voltage applied to wheels
    voltages = [voltagesOut(1); voltagesOut(1); voltagesOut(2); voltagesOut(2)];
    % *-------------------------------------*
    %  YOU CAN ADD/CHANGE YOUR CONTROLS HERE
    % *-------------------------------------*
    
    
    % Run model *** DO NOT CHANGE
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);   
    
    % Euler intergration *** DO NOT CHANGE
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time); 
    time(timeStep + 1)    = timeStep * stepSize_time;
    
    % Plot robot on canvas  *------* YOU CAN ADD STUFF HERE
    figure(1); clf; hold on; grid on; axis([-5,5,-5,5]);
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');
    plot(wall_1(:,1), wall_1(:,2),'k-');
    plot(wall_2(:,1), wall_2(:,2),'k-'); 
    plot(wall_3(:,1), wall_3(:,2),'k-');
    plot(wall_4(:,1), wall_4(:,2),'k-');
    plot(wall_5(:,1), wall_5(:,2),'k-');
    plot(wall_6(:,1), wall_6(:,2),'k-');
    xlabel('y, m'); ylabel('x, m');
end
%% %% Plot results
% *----------------------------------*
%  YOU CAN ADD OR CHANGE FIGURES HERE
%  don't forget to add axis labels!
% *----------------------------------*
fig_x_y = figure(2); hold on; grid on;
xlabel("y-pos")
ylabel("x-pos")
title("path of system using FL controller");
plot(state(:,20), state(:,19));
saveas(fig_x_y, "x_y fl position.jpg");

fig_x_pos = figure(3); hold on; grid on;
xlabel("time (s)");
ylabel("x-pos (m)");
title("x distance travelled");
plot(time, state(:,19));
% saveas(fig_x_pos, "x distance travelled.jpg")

fig_psi = figure(4); hold on; grid on;
xlabel("time (s)");
ylabel("psi (rad)");
title("heading angle (psi)");
plot(time, state(:,24));
% saveas(fig_psi,"heading angle clockwise (psi).jpg")