function [vwave, wwave] = inner_o_wave(mojave)
%INNER_O_WAVE brainwave to turn into and out of the inner O.
    vwave = zeros(1, 13);
    wwave = zeros(1, 13);

    tags = mojave.find_april_tags();
    if isempty(tags)
        return
    end

    ids = zeros(1, length(tags));
    for i = 1:length(tags)
        ids(i) = tags(i).id;
    end

    angs = -30:5:30;
    sigma = 5;
    if any(ids == 4)
        tag = tags(ids == 4);
        if norm(tag.pose.Translation) < 4000
            wwave = wwave + normpdf(angs, 5, sigma);
        end
    end
    if any(ids == 5)
        tag = tags(ids == 5);
        if norm(tag.pose.Translation) < 4000
            wwave = wwave + normpdf(angs, 20, sigma);
        end
    end
    if any(ids == 12)
        tag = tags(ids == 12);
        if norm(tag.pose.Translation) < 6000
            wwave = wwave + normpdf(angs, 10, sigma);
        end
    end
    wwave = wwave / normpdf(0, 0, 5);
end