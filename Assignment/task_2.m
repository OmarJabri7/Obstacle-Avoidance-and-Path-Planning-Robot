close all;
clear all;
clc;
simulationTime_total = 100;
stepSize_time = 0.05;

voltage_left = 6;
voltage_right = 6;

state_initial = zeros(1,24); 

canvasSize_horizontal = 6;
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
figure(1);
alpha = 0.1;
plot(checkpoints(:,2), checkpoints(:,1),'k--d');
hold on;
plot(wall_1(:,1), wall_1(:,2),'k-');
plot(wall_2(:,1), wall_2(:,2),'k-');
plot(wall_3(:,1), wall_3(:,2),'k-');
plot(wall_4(:,1), wall_4(:,2),'k-');
xlim([-5 5])
ylim([-5 5])
avoid = readfis("robot.fis");
% turning = readfis("turning_angle_fl.fis");
% path_following = readfis("path_following.fis");
% path_avoid = readfis("path_avoid_V1.fis");
voltagesOut = [0 0];
sensorBool = zeros(2,30);
sensorCount = 1;
timeSinceLastSensor = 0;
sensorCapture = 0;
timeLimit = 15;
%%
for timeStep = 1:timeSteps_total
    if(sensorCount > length(sensorBool))
        sensorCount = 1;
    end
    sensorOut = Sensor(state(timeStep,19), state(timeStep,20),state(timeStep,24), obstacleMatrix);
    sensorBool(:,sensorCount) = sensorOut;
    sensorLeft = sensorBool(1,:);
    sensorRight = sensorBool(2,:);
    currentLocation = [state(timeStep,19),state(timeStep,20)];
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
%     if(sensorOut(1) < 0.4 || sensorOut(2) < 0.4)
%         sensorCapture = sensorCapture + 1;
%     end
%     if((sensorOut(1) > 0.4 || sensorOut(2) > 0.4) && sensorCapture~=0)
%        timeSinceLastSensor = timeSinceLastSensor + 1;
%     end
%     if(timeSinceLastSensor == timeLimit)
%        timeSinceLastSensor = 0;
%     end
    [booleanAtCheckpoint, newHeadingAngle] = ComputeHeadingAngle(state(timeStep,19:20), checkpoint, tolerance);
%     disp(newHeadingAngle);
    if(booleanAtCheckpoint == 1)
        counterCheckpoint = counterCheckpoint + 1;
    end
    disp(sensorOut);
    %IMPORTANT:
%     correct_angle = turning_angle(currentHeadingAngle,newHeadingAngle);
    correct_angle = limitAngle(currentHeadingAngle,newHeadingAngle);
    disp(correct_angle);
    if(sensorOut(1) < 1 || sensorOut(2) < 1)
        if(sensorOut(1) < 1)
            sensorLeft(sensorCount) = sensorOut(1);
        end
        if(sensorOut(2) < 1)
            sensorRight(sensorCount) = sensorOut(2);
        end
        disp("AVOID");
        voltagesOut = round(evalfis(avoid,sensorOut));
    end
    left_check = sensorLeft == 1;
    left_check = sensorBool(1,:) == 1;
    right_check = sensorRight == 1;
    if(all(left_check) == 1 && all(right_check) == 1)
        deltaAngle = limitAngle(currentHeadingAngle,newHeadingAngle);
        disp("PLAN");
        deltaPosition = checkpoint - currentLocation;
%         [voltagesOut(1), voltagesOut(2)] = pure_pursuit(checkpoint,currentLocation,currentHeadingAngle);
%         voltagesOut = round(evalfis(path_following,[norm(deltaPosition),deltaAngle]));
%         [voltagesOut(1), voltagesOut(2)] = PController(currentHeadingAngle,newHeadingAngle,norm(checkpoint - currentLocation));
        [voltagesOut(1),voltagesOut(2)] = pure_pursuit(checkpoint,currentLocation,currentHeadingAngle);
    end
%     voltagesOut = evalfis(path_avoid,[correct_angle,sensorOut]);
    voltages = [voltagesOut(1); voltagesOut(1); voltagesOut(2); voltagesOut(2)];
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time); 
    time(timeStep + 1) = timeStep * stepSize_time;
    figure(2);
    clf; hold on; grid on; axis([-5,5,-5,5]);
    scatter(checkpoint(2),checkpoint(1));
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');
    plot(wall_1(:,1), wall_1(:,2),'k-');
    plot(wall_2(:,1), wall_2(:,2),'k-');
    plot(wall_3(:,1), wall_3(:,2),'k-');
    plot(wall_4(:,1), wall_4(:,2),'k-');
    xlabel('y, m'); ylabel('x, m');
    sensorCount = sensorCount + 1;
end
%% 
fig_x_y = figure(3); hold on; grid on;
xlabel("y-pos")
ylabel("x-pos")
title("path of system using with obstacles");
plot(state(:,20), state(:,19));
saveas(fig_x_y, "x_y no obstacles position.jpg");
%