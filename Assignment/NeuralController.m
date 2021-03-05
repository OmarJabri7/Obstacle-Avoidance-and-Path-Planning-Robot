function [voltage_left,voltage_right] = NeuralController(sensor_left,sensor_right,params)
T1 = params(1);
T2 = params(2);
w1 = params(3);
w2 = params(4);
w3 = params(5);
w4 = params(6);

if((sensor_left*w1 + sensor_right*w3) > T1)
    voltage_left = 6;
else
    voltage_left = 0;
end
if((sensor_left*w2 + sensor_right*w4) > T2)
    voltage_right = 6;
else
    voltage_right = 0;
end
end

