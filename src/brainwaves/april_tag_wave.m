function [output_vel_vec,output_ang_vec] = april_tag_wave(mojave)
%april_tag_wave Calculate brainwaves based on april tags
%   Tries to keep april tags on the outside of the course to its left
%   and april tags on the inside of the course to its right
    
    INNER_TAGS = [4 5 13 14 15 16];         % Tags on the stone blocks
    OUTER_TAGS = [1 2 3 7 8 9 10 11 12];     % Tags on the pillars
    OVAL_TAGS = [INNER_TAGS OUTER_TAGS];
%     april_tags = evalin('base', 'tags');
    april_tags = mojave.find_april_tags();

    output_vel_vec = [0 0.2 0.5 0.8 1 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05]; % Default vel vec - decent speed, not too fast
    output_ang_vec = zeros(1, 13);
    angs = -30:5:30;               % What the brainwave values represent

    % If no tags, go straight
    num_tags = length(april_tags);
    if num_tags == 0
        output_ang_vec = normpdf(angs, 0, 5);
        return;
    end

    stdev = 5;
    for tag = april_tags
        % Only do stuff if tag is an oval tag (i.e. not gate, depot, etc)
        if sum(OVAL_TAGS == tag.id) >= 1
            % Relevant distance is X. Positive is to the right
            dist = tag.pose.Translation(1);

            % If more than 6000 mm, set to 6000 (with current function,
            % this is where it reaches it maximum turn amount)
            if abs(dist) > 6000
                dist = sign(dist) * 6000;
            end

            % Keep outer tags to left, not right
            side = 1;
            if sum(OUTER_TAGS == tag.id) >= 1
                side = -1;
            end

            % Sample from normal distribution
            % Dist of 3000 is ideal - mean should be 0
            % Dist of 0 should be the lowest (for inner), should map to 30
            mu = (dist - 3000 * side) / 100;
            output_ang_vec = output_ang_vec + normpdf(angs, mu, stdev);
        else
            % If tag is not an oval tag, should not contribute to average
            num_tags = num_tags - 1;
        end
    end
    % Calculate average across each tag, scale up so that the ideal speed
    % gives a brainwave value of 1.
    output_ang_vec = output_ang_vec / num_tags * 1 / normpdf(0, 0, stdev);
end