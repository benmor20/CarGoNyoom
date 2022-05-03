function leave_depot(moj)
%LEAVE_DEPOT Summary of this function goes here
%   Detailed explanation goes here
    steer_angle = -15;
    moj.steer(steer_angle);
    dead_reckon(moj,0.33,1);
    stop_moj(moj,2);
end

