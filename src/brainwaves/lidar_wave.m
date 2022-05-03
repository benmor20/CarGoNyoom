function [vwave, wwave] = lidar_wave(mojave)
    vwave = [0 0.2 0.5 0.7 1 0.9 0.8 0.6 0.4 0.3 0.2 0.1 0];

%     mojave.lidar.tilt_lidar(0);
    scan = mojave.lidar.lidar_scan_single();

    r = scan.Ranges;
    theta = -rad2deg(scan.Angles);

    diff = 10;
    wposs = -60:diff:60;
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

    wwave = meandist / max(meandist);
    wwave = wwave * 2;
end