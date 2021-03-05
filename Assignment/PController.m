function [left_voltage,right_voltage] = PController(currentHeadingAngle,newHeadingAngle,distance,correctAngle)
% if(newHeadingAngle >= currentHeadingAngle)
%     turn_angle = newHeadingAngle - currentHeadingAngle;
%     if(turn_angle > pi)
%         turn_angle = turn_angle - 2*pi;
%     end
% else
%     turn_angle = newHeadingAngle - currentHeadingAngle;
%     if(turn_angle < -pi)
%         turn_angle = turn_angle + 2*pi;
%     end
% end
% if (correctAngle > 180) 
%     angularVelocity = correctAngle - 360 * 60;
% elseif (correctAngle < 180) 
%     angularVelocity = correctAngle + 360 * 60;
% else
%     angularVelocity= correctAngle * 60;
% end
correctAngle = correctAngle * (pi/180);
gain_theta = 4;
gain_delta = 6;
angularVelocity = max(-3*abs(correctAngle) / pi + 1,0);
left_voltage = -correctAngle * gain_theta + distance * angularVelocity * gain_delta;
right_voltage = correctAngle * gain_theta + distance * angularVelocity * gain_delta;
end