function [sonar_lin_vec,sonar_ang_vec] = sonar_brainwave(mojave)
%SONAR_BRAINWAVE Summary of this function goes here
%   Detailed explanation goes here
sonar_lin_vec = [0 0.2 0.5 0.8 1 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05];
sonar_ang_vec = zeros(13,1);
sonar_reading = read_sonar(sonar_scan());
close_threshold;
if sonar_reading < close_threshold % mm
    sonar_ang_vec = []
end 
end

