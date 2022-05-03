function run_robot(moj, arb, time)

    if time <= 0
        time = 15;
    end
    time = time * 60;
    tic;
    while toc < time
%         [vwave, wwave] = arb.get_waves()
        [v, w] = arb.arbitrate();
        moj.set_speed(v);
        moj.steer(w);
        toc
    end
    moj.set_speed(0);
end

