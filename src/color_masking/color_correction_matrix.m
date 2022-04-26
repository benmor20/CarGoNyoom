function ccm = color_correction_matrix(esfr_img)
%COLOR_CORRECTION_MATRIX calculates the optimal color correction matrix
%given an image of the eSFR chart from Imatest.
%   See also apply_color_correction

    chart = esfrChart(esfr_img);
    [colorTable, ~] = measureColor(chart);

    % Get reference to gray, convert to RGB from L*a*b*
    referenceRGB = lab2rgb([chart.ReferenceColorLab; chart.ReferenceGrayLab], 'OutputType', 'uint8');
    
    % Get mean gray value by measuring noise
    noiseTable = measureNoise(chart);
    measuredGrayRGB = [noiseTable.MeanIntensity_R, noiseTable.MeanIntensity_G, noiseTable.MeanIntensity_B];
    
    % Concatenate RGB of all the patches (color & grayscale)
    measuredColorRGB = [colorTable.Measured_R, colorTable.Measured_G, colorTable.Measured_B];
    measuredRGB = [measuredColorRGB; measuredGrayRGB];
    ccm = double([measuredRGB ones(36,1)]) \ double(referenceRGB);
end