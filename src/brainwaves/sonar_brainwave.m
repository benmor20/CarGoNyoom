function [sonar_lin_vec,sonar_ang_vec] = sonar_brainwave(mojave)
    %SONAR_BRAINWAVE Summary of this function goes here
    %   Detailed explanation goes here
    sonar_lin_vec = [0 0.2 0.5 0.8 1 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05];
    ang_vec = -30:5:30;
    distance_to_wall = mojave.sonar_scan();
    close_threshold;
    sigma = 5;
    sonar_ang_vec = normpdf(ang_vec,0,sigma); % arbitrary sigma 
    if distance_to_wall < close_threshold % mm
        % distance of 1000 is ideal
        % we want mu to be 0
        mu = (distance_to_wall - 1000) / -50;
        sonar_ang_vec = sonar_ang_vec + normpdf(angs,mu,sigma);
    else 
        mu = (distance_to_wall - 1000) / 50;
        sonar_ang_vec = sonar_ang_vec + normpdf(angs,mu,sigma);
    end 
    sonar_ang_vec = sonar_ang_vec / normpdf(0, 0, sigma);
end

