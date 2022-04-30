function [output_vel_vec,output_w_vel_vec] = bridge_wave()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
output_vel_vec = [];
output_w_vel_vec = [];
lidar_tilt = 20; 
lidar_data = mojave.lidar.scan(); 
nearest_edge = find_nearest_edge();
proximity_to_edge = mojave.lidar.distance_to_nearest_edge();
threshold; 
if proximity_to_edge < threshold 
    % send negative values
end 

outputArg1 = inputArg1;
outputArg2 = inputArg2;
end