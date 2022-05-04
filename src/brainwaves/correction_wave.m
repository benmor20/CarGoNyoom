function [vwave, wwave] = correction_wave(moj)
    vwave = zeros(size(1, 13));
    wwave = normpdf(-30:5:30, -moj.last_angle, 5) / normpdf(0, 0, 5) * 0.25;
end

