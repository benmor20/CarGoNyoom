function [output_vel_vec,output_ang_vec] = april_tag_wave()
%april_tag_wave Summary of this function goes here
%   Detailed explanation goes here
    INNER_TAGS = [4 5 13 14 15 16];
    OUTER_TAGS = [1 2 3 7 8 9 10 11 12];
    april_tags = mojave.find_april_tags();
end