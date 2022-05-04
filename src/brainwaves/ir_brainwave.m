function [vel_wave,ang_wave] = ir_brainwave(mojave)
    distances = mojave.ir_scan();
    left_dist = distances(1);
    right_dist = distances(2);

    near_dist = 5;
    far_dist = 8.5;

    if left_dist > far_dist || right_dist > far_dist
        vel_wave = [10 zeros(1, 12)];
        ang_wave = zeros(1, 13);
        return
    end


    vel_wave = zeros(1, 13);
    turn_right = normpdf(-30:5:30, 30, 5) / normpdf(0, 0, 5);
    ang_wave = zeros(1, 13);
    if left_dist < near_dist
        if right_dist < near_dist
            vel_wave = [0 0.5 2 1.5 1 0.9 0.7 0.5 0.4 0.3 0.2 0.1 0];
            ang_wave = turn_right * 2;
        else
            ang_wave = turn_right;
        end
    elseif right_dist < near_dist
        ang_wave = flip(turn_right);
    end
end