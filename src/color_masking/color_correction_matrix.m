function ccm = color_correction_matrix(esfr_img)
%COLOR_CORRECTION_MATRIX calculates the optimal color correction matrix
%given an image of the eSFR chart from Imatest.
%   See also apply_color_correction
    [~, ccm] = measureColor(esfrChart(esfr_img));
end