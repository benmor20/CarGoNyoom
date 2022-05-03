%Must clear serial link
classdef arbiter
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        brainwave_vec
        vel_result_vec
        ang_result_vec
        mojave
    end

    methods
        function obj = arbiter(brainwave_vec, vel_result_vec, ang_result_vec, mojave)
            obj.brainwave_vec = brainwave_vec;
            obj.vel_result_vec = vel_result_vec;
            obj.ang_result_vec = ang_result_vec;
            obj.mojave = mojave;

        end

        function [vel_wave, ang_wave] = get_wave_sum(obj)
            [vwaves, awaves] = obj.get_waves();
            vel_wave = sum(vwaves, 1);
            ang_wave = sum(awaves, 1);

            leg = ["Wave 1"];
            for i = 2:length(obj.brainwave_vec)
                leg(i) = "Wave " + i;
            end
            leg(length(leg) + 1) = "Total";
            try
                ang_fig = evalin("base", "ang_fig");
                vel_fig = evalin("base", "vel_fig");

                figure(ang_fig);
                clf;
                plot(obj.ang_result_vec, awaves);
                hold on;
                plot(obj.ang_result_vec, ang_wave);
                ylim([0 5]);
                legend(leg);

                figure(vel_fig);
                clf;
                plot(obj.vel_result_vec, vwaves);
                hold on;
                plot(obj.vel_result_vec, vel_wave);
                ylim([0 5]);
                legend(leg);
            catch
            end

        end

        function [vel_vec,ang_vec] = get_waves(obj)
            vel_vec = zeros(length(obj.brainwave_vec), length(obj.vel_result_vec));
            ang_vec = zeros(length(obj.brainwave_vec), length(obj.ang_result_vec));
            for i = 1:length(obj.brainwave_vec)
                wave_func = obj.brainwave_vec{i};
                [vel, ang] = wave_func(obj.mojave);
                vel_vec(i, :) = vel;
                ang_vec(i, :) = ang;
            end
        end 

        function [vel_result, ang_result] = arbitrate_from_waves(obj, total_vel, total_ang)
            [~, Ivel] = max(total_vel);
            [~, Iang] = max(total_ang);
            vel_result = obj.vel_result_vec(Ivel(1));
            ang_result = obj.ang_result_vec(Iang(1));
        end

        function [vel_result, ang_result] = arbitrate(obj)
            [total_vel, total_ang] = obj.get_wave_sum();
            [vel_result, ang_result] = obj.arbitrate_from_waves(total_vel, total_ang);
        end
    end 
end 