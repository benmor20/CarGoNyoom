function [output_vel_vec, output_ang_vec] = follow_color(mojave, mask_func)
%PATH_WAVE Brainwave to keep path in the middle of the screen

    output_vel_vec = [0 0.2 0.5 1 0.95 0.9 0.8 0.6 0.5 0.3 0.2 0.1 0.05]; % Default vel vec - decent speed, not too fast
    output_ang_vec = zeros(13, 1);

    % Find the mask for the path
    img = mojave.sense_cam();
    mask = mask_func(img);

    % Split the mask into 13 segments
    min_height = 800;
    max_height = 1200;  % Top of image is 0
    prevx = 1;
    xs = round(linspace(size(mask, 2)/13, size(mask, 2), 13));
    for i = 1:13
        mask_slice = mask(min_height:max_height, prevx:xs(i));
        % Resulting ang vec is proportional to amount of color in that segment
        output_ang_vec(i) = sum(mask_slice(:));
        prevx = xs(i);
    end
    
    % Scale down so most beige goes to brainwave val of 1
    output_ang_vec = output_ang_vec * 1 / max(output_ang_vec);
end

