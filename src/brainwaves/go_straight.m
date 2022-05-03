function [vwave,wwave] = go_straight(~)
%go_straight Brainwave for the robot to go straight
    vwave = [0 0.2 0.5 0.8 1 1.1 0.7 0.5 0.4 0.3 0.2 0.1 0];
    wwave = normpdf(-30:5:30, 0, 5) * 0.25 / normpdf(0, 0, 5);
end