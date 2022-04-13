function april_tags = find_april_tags(img, intrinsics)
    % Find and return all the april tags in an image

    [ids, corners, poses] = readAprilTag(img, 'tag36h11', intrinsics, 164);
    if isempty(ids)
        april_tags = [];
    else
        april_tags = [april_tag(ids(1), corners(:, :, 1), poses(1))];
        for i = 2:length(ids)
            april_tags(i) = april_tag(ids(i), corners(:, :, i), poses(i));
        end
    end
end