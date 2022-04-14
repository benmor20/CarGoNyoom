function [] = plot_image(uiaxes, masked_image)
%PLOT_IMAGE plots an image on the given axes
%   Plots the BW image on the given axes
    
    % Convert to RGB
    if ismatrix(masked_image) % if it's 2D
        scaled = masked_image * 255;
        rgb = zeros([size(masked_image), 3]);
        for i = 1:3
            rgb(:, :, i) = scaled;
        end
    else
        rgb = masked_image;
    end

    % Plot masked image
    image(uiaxes, rgb);
end

