function [outputArg1,outputArg2] = lidar_bridge(arduino)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
tilt_angle = 10;
writePosition(obj.lidar.tilt_servo,tilt_angle)
outputArg1 = inputArg1;
outputArg2 = inputArg2;
end