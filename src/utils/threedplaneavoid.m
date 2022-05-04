function threedplaneavoid(moj)
scan = moj.lidar.lidar_scan_3d;
[plane_model, inliers, outliers] = pcfitplane(scan,1000,[1 0 0],pi/12);
plane = select(scan, inliers);
figure(5);
pcshow(scan);
hold off
figure(6);
pcshow(plane);
end