function [sonar_lin_vec,sonar_ang_vec] = sonar_brainwave(mojave)
    %SONAR_BRAINWAVE Summary of this function goes here
    %   Detailed explanation goes here
    sonar_lin_vec = zeros(1, 13);
    angs = -30:5:30;
    distances = mojave.sonar_scan();
    distance_to_wall = distances(1)
    
    sonar_ang_vec = zeros(1, 13);
    sigma = 5;
    if distance_to_wall < 4
        sonar_ang_vec = normpdf(angs, 15, sigma) / normpdf(0, 0, sigma);
    else
        sonar_ang_vec = normpdf(angs, -15, sigma) / normpdf(0, 0, sigma);
    end
end

