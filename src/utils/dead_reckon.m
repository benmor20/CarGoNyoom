function dead_reckon(moj,mojave_speed,duration)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    tic;
    while toc < duration
       moj.set_speed(mojave_speed);
       toc
    end 
end

