function [output_vel_vec,output_w_vel_vec] = bridge_wave(mojave)
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here
    output_vel_vec = zeros(1,13);
    output_w_vel_vec = zeros(1,13);
    lidar_tilt = 20; 
    lidar_data = mojave.lidar.scan(); 
    nearest_edge = find_nearest_edge();
    proximity_to_edge = mojave.lidar.distance_to_nearest_edge();
    threshold; 
    if proximity_to_edge < threshold 
        % send negative values
    end 
end