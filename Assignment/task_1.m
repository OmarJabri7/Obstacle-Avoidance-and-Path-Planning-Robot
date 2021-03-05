close all;
clear all;
clc;
simulationTime_total = 100;
stepSize_time = 0.05;

voltage_left = 6;
voltage_right = 6;

state_initial = zeros(1,24); 

canvasSize_horizontal = 10;
canvasSize_vertical   = 10;
stepSize_canvas       = 0.01;

obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);

timeSteps_total = simulationTime_total/stepSize_time;
state = state_initial;
time = 0;
%% Task 1:
tolerance = 0.2;
counterCheckpoint = 1;
checkpoints = [0 3;1 2;-1 4;-1 -2;-0.2 2];
robotInitialLocation = checkpoints(1,:);
robotGoal = checkpoints(end,:);
figure
plot(checkpoints(:,2), checkpoints(:,1),'k--d')
xlim([-5 5])
ylim([-5 5])
path_planning = readfis("turning_angle_fl_new.fis");
% path_following = readfis("path_following.fis");
voltagesOut = [];
%%
for timeStep = 1:timeSteps_total
    disp(counterCheckpoint);
    currentHeadingAngle = state(timeStep,24);
    currentLocation = [state(timeStep,19),state(timeStep,20)];
    if(counterCheckpoint <= length(checkpoints))
        checkpoint = [checkpoints(counterCheckpoint,:)];
    else
        break;
    end
    [booleanAtCheckpoint, newHeadingAngle] = ComputeHeadingAngle(state(timeStep,19:20), checkpoint, tolerance);
    if(booleanAtCheckpoint == 1)
        counterCheckpoint = counterCheckpoint + 1;
    end
    correct_angle = turning_angle(currentHeadingAngle,newHeadingAngle);
    deltaAngle = newHeadingAngle - currentHeadingAngle;
    deltaPosition = checkpoint - currentLocation;
%     [voltagesOut(1), voltagesOut(2)] = PController(currentHeadingAngle,newHeadingAngle,norm(checkpoint - currentLocation),correct_angle);
%     voltagesOut = round(evalfis(path_following,[norm(deltaPosition),deltaAngle]));
    voltagesOut = round(evalfis(path_planning,correct_angle));
%     [voltagesOut(1),voltagesOut(2)] = pure_pursuit(checkpoint,currentLocation,currentHeadingAngle);
%     [voltagesOut(1),voltagesOut(2)] = left_right_speed(10,correct_angle);
    disp(correct_angle);
%     disp(norm(deltaPosition));
%     disp(voltagesOut);
    voltages = [voltagesOut(1); voltagesOut(1); voltagesOut(2); voltagesOut(2)];
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);   
    
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time); 
    time(timeStep + 1)    = timeStep * stepSize_time;
%     if()
    figure(1);
    clf; hold on; grid on; axis([-5,5,-5,5]);
    scatter(checkpoint(2),checkpoint(1));
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');
    xlabel('y, m'); ylabel('x, m');
end
%% 
fig_x_y = figure(2); hold on; grid on;
xlabel("y-pos")
ylabel("x-pos")
title("path of system using without obstacles");
plot(state(:,20), state(:,19));
saveas(fig_x_y, "x_y no obstacles position.jpg");
% 
% fig_x_pos = figure(3); hold on; grid on;
% xlabel("time (s)");
% ylabel("x-pos (m)");
% title("x distance travelled");
% plot(time, state(:,19));
% % saveas(fig_x_pos, "x distance travelled.jpg")
% 
% fig_psi = figure(4); hold on; grid on;
% xlabel("time (s)");
% ylabel("psi (rad)");
% title("heading angle (psi)");
% plot(time, state(:,24));
% % saveas(fig_psi,"heading angle clockwise (psi).jpg")