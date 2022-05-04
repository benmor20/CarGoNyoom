function [lin_vel_vec,ang_vel_vec] = lidar_inside_brainwave(mojave)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    lin_vel_vec = [0 0.2 0.5 0.8 1 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05];
    theta_real = -30:5:30;
    scan = mojave.lidar.lidar_scan_single();
    ranges = scan.Ranges;
    theta = rad2deg(scan.Angles);
    filtered = ranges < 6000 & ranges > 200;
%     filtered = ones(1, length(theta), 'logical');
    theta = theta(filtered);
    ranges = ranges(filtered);
    [x, y] = pol2cart(deg2rad(theta), ranges);
    cart = [x y];

    try
        fig = evalin("base", "lidar_fig");
        figure(fig);
        hold on;
        plot(cart(:, 1), cart(:, 2), '.k', 'MarkerSize', 10);
        hold off;
    catch
    end

    fit_line_func = @(points) polyfit(points(:,1),points(:,2),1);
    eval_line_func = @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);
    min_points = 10;
    max_delta = 200;
    has_model = false;
    try
        model = [20 0];
        inliers = zeros(length(cart), 1, 'logical');
        while abs(model(1)) > 1 && length(cart) > 15
            cart = cart(~inliers, :);
            [model, inliers] = ransac(cart, fit_line_func, eval_line_func, min_points, max_delta);
            model;
        end
        if length(cart) <= 15
            ang_vel_vec = zeros(1, 13);
            return
        end
        has_model = true;
    catch ERROR
        switch ERROR.identifier
            case 'vision:ransac:notEnoughInliers'
                ang_vel_vec = zeros(1, 13);
                return
            otherwise
                disp(ERROR.identifier);
                rethrow(ERROR);
        end
    end
    if ~has_model
        return
    end

    inside_points = cart(inliers, :);
%     inliers = dbscan(inside_points, 500, 3);
%     inliers = inliers > -1;
%     inside_points = inside_points(inliers, :);
    midpoint = (inside_points(1) + inside_points(end)) / 2;
    dist = abs(midpoint(1));

    try
        fig = evalin("base", "lidar_fig");
        figure(fig);
        hold on;
        plot(inside_points(:, 1), inside_points(:, 2), '.m', 'MarkerSize', 10);
        hold off;
    catch
    end

    wall_angle = 3*atand(model(1));
    ang_vel_vec = normpdf(-30:5:30, wall_angle, 5) / normpdf(0, 0, 5) * 0.5;


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