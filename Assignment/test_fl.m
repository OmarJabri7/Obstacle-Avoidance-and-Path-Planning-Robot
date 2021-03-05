robot = readfis("robot.fis");
input = [0.2,0.3];
voltages = round(evalfis(robot,input));
disp(voltages);