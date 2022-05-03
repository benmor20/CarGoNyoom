function [lin_vel_vec,ang_vel_vec] = lidar_inside_brainwave(mojave, correction_dist)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    lin_vel_vec = [0 0.2 0.5 0.8 1 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05];
    theta_real = -30:5:30;
    scan = mojave.lidar.lidar_scan_single();
    ranges = scan.Ranges;
    theta = -rad2deg(scan.Angles);
    filtered = theta >= 30 & ranges < 5000 & ranges > 200;
    theta = theta(filtered);
    ranges = ranges(filtered);
    cart = pol2cart(theta, ranges);

    fit_line_func = @(points) polyfit(points(:,1),points(:,2),1);
    eval_line_func = @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);
    min_points = 10;
    max_delta = 200;
    try
        model = [0 0];
        while abs(model(1)) < 1
            [model, inliers] = ransac(cart, fit_line_func, eval_line_func, min_points, max_delta);
        end
    catch
        ang_vel_vec = zeros(1, 13);
        return
    end

    inside_points = cart(inliers);
    midpoint = (inside_points(1) + inside_points(end)) / 2;
    dist = abs(midpoint(1));

    scale = 30 / (c - 1000);
    mu = scale * (dist - correction_dist);
    ang_vel_vec = normpdf(-30:5:30, mu, 5);


%     close_threshold = 2500; % mm
%     sigma = 5;
%     % we want mu to be 0 
%     ang_vel_vec = zeros(size(theta_real));
%     med = median(ranges);
%     if ~isnan(med)
%         mu = (median(ranges) - close_threshold) / 10;
%         ang_vel_vec = ang_vel_vec + normpdf(theta_real,mu,sigma);
%         ang_vel_vec = ang_vel_vec * 2 / normpdf(0, 0, sigma);
%     end
end