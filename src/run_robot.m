function run_robot(moj, arb, finished_func)
    if isnumeric(finished_func)
        time = finished_func;
        if time <= 0
            time = 15;
        end
        time = time * 60;

        finished_func = @(m) until_time(m, time);
        tic;
    end

    while ~finished_func(moj)
%         [vwave, wwave] = arb.get_waves()
        [v, w] = arb.arbitrate();
        moj.set_speed(v);
        moj.steer(w);
        toc
    end
    moj.set_speed(0);
end

function done = until_time(~, t)
    toc
    done = toc > t;
end

