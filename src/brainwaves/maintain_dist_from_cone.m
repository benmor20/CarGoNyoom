function [output_vel_vec, output_ang_vec] = maintain_dist_from_cone(mojave, cone_detection)
%PATH_WAVE Brainwave to keep cone in particular segment of image 

    output_vel_vec = [0 0.2 0.5 1 0.95 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05]; % Default vel vec - decent speed, not too fast
    output_ang_vec = zeros(13, 1);

    % Find the mask for the path
    img = mojave.sense_cam();
    mask = cone_detection(img);
    ang_vec = -30:5:30;
    % Split the mask into 13 segments
    segment_count = 3;
    min_height = 400;
    max_height = 1200;  % Top of image is 0
    prevx = 1;
    segment_orange_counts = [];
    max_orange_count = 0;
    segment_num = 0;
    xs = round(linspace(size(mask, 2)/segment_count, size(mask, 2), segment_count));
    for i = 1:segment_count
        mask_slice = mask(min_height:max_height, prevx:xs(i));
        % Resulting ang vec is proportional to amount of color in that segment
        segment_orange_counts(i) = sum(mask_slice(:));
        if segment_orange_counts(i) > max_orange_count
            max_orange_count = segment_orange_counts(i);
            segment_num = i;
        end
        prevx = xs(i);
    end
    desired_heading = 15;
    mu = desired_heading * sign(segment_orange_counts(3) - segment_orange_counts(1));
    sigma = 5;
    output_ang_vec = normpdf(ang_vec,mu,sigma);
    
    % Scale down so most beige goes to brainwave val of 1
    output_ang_vec = output_ang_vec * 1 / normpdf(0,0,sigma);
end

