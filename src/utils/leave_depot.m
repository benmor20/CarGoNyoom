function leave_depot(moj)
%LEAVE_DEPOT Summary of this function goes here
%   Detailed explanation goes here
    steer_angle = -20;
    dead_reckon(0.5,2);
    stop_moj(moj,2);
    moj.steer(steer_angle);
    dead_reckon(0.5,3);
    stop_moj(moj,2);
    moj.steer(0);
    dead_reckon(0.5,4);
end

