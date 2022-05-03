function [lin_vel,ang_pos] = lidar_arch(mojave)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    tilt_angle = 0;
    mojave.lidar.tilt_lidar(tilt_angle);
    scan = mojave.lidar.lidar_scan_single();
    ranges = scan.Ranges;
    angs = rad2deg(scan.Angles);
    filter = angs < 30 & angs > -30;
    angs_filtered = angs(filter);
    ranges = ranges(filter);
    close_threshold = 76.2; % mm 
    sigma = 5;
    theta_real = -30:5:30;
    ang_pos = normpdf(theta_real,0,sigma);
    if any(ranges <= 381)
        mu = (mean(ranges) - 152.4) / -15.2;
        ang_pos = ang_pos + normpdf(theta_real,mu,sigma);
    else
        mu = (mean(ranges) - 152.4) / 15.2;
        ang_pos = ang_pos + normpdf(theta_real,mu,sigma);
    end 
    ang_pos = ang_pos / normpdf(0, 0, sigma);
    lin_vel = [0 0.2 0.5 2 1 0.8 0.6 0.5 0.4 0.3 0.2 0.1 0];
end