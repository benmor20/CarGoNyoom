function [lin_vel,ang_pos] = lidar_bridge(arduino)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
tilt_angle = 10;
writePosition(obj.lidar.tilt_servo,tilt_angle);
scan = lidar.lidar_scan_single()
ranges = scan.Ranges;
angs = rad2deg(scan.Angles);
angs_filtered = angs < 30;
angs_filtered = angs > -30;
ranges = angs < 30;
ranges = angs > -30;
close_threshold = 76.2; % mm 
sigma = 5;
    ang_pos = normpdf(theta_real,0,sigma);
if any(ranges <= 381)
    mu = (ranges - 152.4) / -15.2;
    ang_pos = ang_pos + normpdf(theta_real,mu,sigma);
else
    mu = (ranges - 152.4) / 15.2;
    ang_pos = ang_pos + normpdf(theta_real,mu,sigma);
end 
    ang_pos = ang_pos / normpdf(0, 0, sigma);
end