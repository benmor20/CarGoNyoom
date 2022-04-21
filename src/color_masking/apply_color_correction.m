function Icc = apply_color_correction(I, ccm)
%APPLY_COLOR_CORRECTION applies the given color calibration matrix to the
%given image.

    % No idea what this does, don't question it
    % Code copied from
    % https://www.mathworks.com/help/images/correct-colors-using-color-correction-matrix.html
    Icc = imapplymatrix(ccm(1:3,:)',I,ccm(4,:));
end