function [output_vel_vec,output_ang_vec] = park_brainwave(moj)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    output_vel_vec = [0 0.2 0.5 0.8 1 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05]; % Default vel vec - decent speed, not too fast
    output_ang_vec = zeros(1, 13);
    angs = -30:5:30;               % What the brainwave values represent
    april_tags = moj.find_april_tags();
    sigma = 5;
    % If no tags, go straight
    num_tags = length(april_tags);
    if num_tags == 0
        output_ang_vec = normpdf(angs, 0, 5);
        return;
    end

    for tag = april_tags
        if tag.id == 0
            dist = tag.pose.Translation(1) * 1000;
            mu = (dist - 0) / -16;
            output_ang_vec = normpdf(angs,mu,sigma);
        end 
    end
    output_ang_vec = output_ang_vec / normpdf(0, 0, sigma);
end