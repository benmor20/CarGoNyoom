function [vwave, wwave] = lidar_wave(mojave)
    vwave = [0 0.2 0.5 0.7 1 0.9 0.8 0.6 0.4 0.3 0.2 0.1 0];

%     mojave.lidar.tilt_lidar(0);
    scan = mojave.lidar.lidar_scan_single();

    r = scan.Ranges;
    theta = -rad2deg(scan.Angles);

    wposs = linspace(-90, 90, 13);
    diff = wposs(2) - wposs(1);
    meandist = zeros(size(wposs));
    for i = 1:length(wposs)
        ang = wposs(i);
        filtered_dists = r(abs(theta - ang) < diff / 2);
        if isempty(filtered_dists)
            meandist(i) = 10000;
        else
            meandist(i) = mean(filtered_dists);
        end
    end

%     [~, idx] = min(meandist);
%     angs = -30:5:30;
%     sigma = 5;
%     if idx == 1
%         wwave = normpdf(angs, 15, sigma);
%     elseif idx == 3
%         wwave = normpdf(angs, -15, sigma);
%     else
%         wwave = normpdf(angs, -15, sigma) + normpdf(angs, 15, sigma);
%     end
%     wwave = wwave / normpdf(0, 0, sigma);


    leftpoints = [meandist(2:end) 0];
    rightpoints = [0 meandist(1:end-1)];
    total = leftpoints + meandist + rightpoints;

    wwave = total / max(total);
    wwave = wwave * 2;
end