function [outputArg1,outputArg2] = threedplaneavoid(inputArg1,inputArg2)
scan = moj.lidar.lidar_scan_3d;
plane = pcfitplane(scan,1000,[0 1 0],pi/12);
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
pcshow(scan);
hold off
pcshow(plane);
outputArg1 = inputArg1;
outputArg2 = inputArg2;
end