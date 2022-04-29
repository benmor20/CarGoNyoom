function [output_vel_vec,output_ang_vec] = april_tag_wave()
%april_tag_wave Summary of this function goes here
%   Detailed explanation goes here
    INNER_TAGS = [4 5 13 14 15 16];
    OUTER_TAGS = [1 2 3 7 8 9 10 11 12];
    OVAL_TAGS = [INNER_TAGS OUTER_TAGS];
    april_tags = mojave.find_april_tags();

    num_tags = length(april_tags);
    output_ang_vec = zeros(7, 1);
    angs = -30:10:30;
    for tag = april_tags
        if sum(OVAL_TAGS == tag.id) > 1
            dist = tag.pose.Translation(1);
            scale = 1;
            if sum(OUTER_TAGS == tag.id) > 1
                scale = -1;
            end

            normx = -30:1:30;
            normdist = normpdf(x, scale * (dist - 3000) / 100, 5);
            output_ang_vec = normdist(sum(normx == angs) == 1);
        else
            num_tags = num_tags - 1;
        end
    end

    output_vel_vec = linspace(0, 1, 7);
end