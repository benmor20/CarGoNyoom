function [vel_wave,ang_wave] = ir_brainwave(mojave)
distances = mojave.ir_scan();
near_threshold
far_threshold
for i = 1:4 
    if distances(i) < near_threshold || distances(i) > far_threshold
        vel_wave = [10 0 0 0 0 0 0 0 0 0 0 0 0];
    else
        vel_wave = [0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    end
    ang_wave = [0 0 0 0 0 0 0 0 0 0 0 0 0 0];
end