function stop_moj(moj,stop_time)
%STOP_MOJ Summary of this function goes here
%   Detailed explanation goes here
    moj.set_speed(0);
    pause(stop_time);
end

