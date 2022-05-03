function [lin_vel_vec,ang_vel_vec] = lidar_inside_brainwave(mojave)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    lin_vel_vec = [0 0.2 0.5 0.8 1 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05];
    theta_real = -30:5:30;
    scans = mojave.lidar.lidar_scan_single();
    ranges = scans.Ranges;
    theta = -rad2deg(scans.Angles);
    ranges = ranges(theta >= 30);
    ranges = ranges(ranges < 4000 & ranges > 200);
    close_threshold = 2500; % mm
    sigma = 5;
    % we want mu to be 0 
    ang_vel_vec = zeros(size(theta_real));
    med = median(ranges);
    if ~isnan(med)
        mu = (median(ranges) - close_threshold) / 10;
        ang_vel_vec = ang_vel_vec + normpdf(theta_real,mu,sigma);
        ang_vel_vec = ang_vel_vec * 2 / normpdf(0, 0, sigma);
    end
end