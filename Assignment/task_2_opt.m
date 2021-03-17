close all;
clear all;
clc;
simulationTime_total = 200;
stepSize_time = 0.05;

voltage_left = 6;
voltage_right = 6;

state_initial = zeros(1,24); 

canvasSize_horizontal = 10;
canvasSize_vertical   = 10;
stepSize_canvas       = 0.01;

obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);
[wall_1, obstacleMatrix] = WallGeneration( -1,  1, 1.2, 1.2, 'h', obstacleMatrix);
[wall_2, obstacleMatrix] = WallGeneration( -3,  1, 4, 4, 'h', obstacleMatrix);
[wall_3, obstacleMatrix] = WallGeneration( -3, -3,  -2,   2, 'v', obstacleMatrix);
[wall_4, obstacleMatrix] = WallGeneration(2,2,-3,1,'v',obstacleMatrix);
timeSteps_total = simulationTime_total/stepSize_time;
state = state_initial;
time = 0;
%% Task 2:
tolerance = 0.1;
counterCheckpoint = 1;
checkpoints = [-2 3;-1 -4;3 -4;-1 -2;3 0; 3.5 2.5];
figure;
alpha = 0.1;
plot(checkpoints(:,2), checkpoints(:,1),'k--d');
xlim([-5 5])
ylim([-5 5])
obstacle_avoidance = readfis("robot_2.fis");
path_planning = readfis("turning_angle_fl_new.fis");
% weights = readfis("f_w.fis");
voltagesOut = [0 0];
memory = 0;
%%
for timeStep = 1:timeSteps_total
    sensorOut = Sensor(state(timeStep,19), state(timeStep,20),state(timeStep,24), obstacleMatrix);
    if(timeStep == 1)
        state(timeStep,19) = -2;
        state(timeStep,20) = -1;
    end
    currentHeadingAngle = state(timeStep,24);
    if(counterCheckpoint <= length(checkpoints))
        checkpoint = [checkpoints(counterCheckpoint,:)];
    else
        break;
    end
    disp(sensorOut);
    [booleanAtCheckpoint, newHeadingAngle] = ComputeHeadingAngle(state(timeStep,19:20), checkpoint, tolerance);
    correct_angle = turning_angle(currentHeadingAngle,newHeadingAngle);
    if(sensorOut(1) < 1 || sensorOut(2) < 1)
        memory = 1;
    else
        memory = memory*0.95;
    end
    if(memory > 0.4)
        voltagesOut = round(evalfis(obstacle_avoidance,sensorOut));
    else
        voltagesOut = evalfis(path_planning,correct_angle); 
    end
    disp(memory);
    if(booleanAtCheckpoint == 1)
        counterCheckpoint = counterCheckpoint + 1;
    end
    voltages = [voltagesOut(1); voltagesOut(1); voltagesOut(2); voltagesOut(2)];
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time); 
    time(timeStep + 1)    = timeStep * stepSize_time;
    figure(1);
    clf; hold on; grid on; axis([-5,5,-5,5]);
    scatter(checkpoint(2),checkpoint(1));
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');
    plot(wall_1(:,1), wall_1(:,2),'k-');
    plot(wall_2(:,1), wall_2(:,2),'k-'); 
    plot(wall_3(:,1), wall_3(:,2),'k-');
    plot(wall_4(:,1), wall_4(:,2),'k-');
    xlabel('y, m'); ylabel('x, m');
end
%% 
fig_x_y = figure(2); hold on; grid on;
xlabel("y-pos")
ylabel("x-pos")
title("path of system using with obstacles");
plot(state(:,20), state(:,19));
saveas(fig_x_y, "x_y avoid and plan.jpg");
%