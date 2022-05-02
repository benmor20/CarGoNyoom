function [lin_vel_vec,ang_vel_vec] = lidar_inside_brainwave(mojave)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    lin_vel_vec = [0 0.2 0.5 0.8 1 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05];
    angs = -30:5:30;
    scans = obj.lidar.lidar_scan_single();
    ranges = scans.Ranges;
    theta = -rad2deg(scan.Angles);
    theta_real = (theta >= 30);
    ranges = (theta >= 30);
    close_threshold = 381; % mm 
    sigma = 5;
    % we want mu to be 0 
    ang_vel_vec = normpdf(theta_real,0,sigma);
    if any(ranges <= 381)
        mu = (ranges - 381) / -12;
        ang_vel_vec = ang_vel_vec + normpdf(theta_real,mu,sigma);
    else
        mu = (ranges - 381) / 12;
        ang_vel_vec = ang_vel_vec + normpdf(theta_real,mu,sigma);
    end 
    ang_vel_vec = ang_vel_vec / normpdf(0, 0, sigma);
end