function [v_l,v_r] = left_right_speed(r,theta)
theta = theta * (180/pi);
theta = (mod((theta + 180),360) - 180);
r = min(max(0,r),100);
v_a = r*(45 - mod(theta,90))/45;
minim = min(100,2*r+v_a);
v_b = min(minim,2*r-v_a);
if(theta < -90)
    v_l = -v_b;
    v_r = -v_a;
elseif(theta < 0)
    v_l = -v_a;
    v_r = v_b;
elseif(theta < 90)
    v_l = v_b;
    v_r = v_a;
else
    v_l = v_a;
    v_r = -v_b;
end
end

