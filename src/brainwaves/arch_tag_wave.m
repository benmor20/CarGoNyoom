function [vwave, wwave] = arch_tag_wave(mojave)
%ARCH_TAG_WAVE Brainwave for the arch based on the tag

    arch_id = 17;
    vwave = zeros(1, 13);
    wwave = zeros(1, 13);

    tags = mojave.find_april_tags();
    if isempty(tags)
        return
    end

    ids = zeros(size(tags));
    for i = 1:length(tags)
        ids(i) = tags(i).id;
    end
    if ~any(ids == arch_id)
        return
    end

    arch_tag = tags(ids == arch_id);
    dist = arch_tag.pose.Translation(1);
    mu = (dist - 350) / 100;
    if abs(mu) > 30
        mu = sign(mu) * 30;
    end
    wwave = normpdf(-30:5:30, mu, 5);
end

