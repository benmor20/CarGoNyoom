function [sonar_lin_vec,sonar_ang_vec] = sonar_brainwave(mojave)
    %SONAR_BRAINWAVE Summary of this function goes here
    %   Detailed explanation goes here
    sonar_lin_vec = [0 0.2 0.5 0.8 1 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05];
    angs = -30:5:30;
    distances = mojave.sonar_scan();
    distance_to_wall = distances(1)
    sigma = 5;
    sonar_ang_vec = zeros(1, 13);
    mu = (distance_to_wall - 30) / -2;
    sonar_ang_vec = sonar_ang_vec + normpdf(angs,mu,sigma);
    sonar_ang_vec = sonar_ang_vec * 0.5 / normpdf(0, 0, sigma);
end

