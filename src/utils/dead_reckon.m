function dead_reckon(moj,mojave_speed,duration)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    start_time = tic;
    curr_time = start_time;
    while curr_time > duration
       moj.set_speed(mojave_speed);
       curr_time = toc;
    end 
end

