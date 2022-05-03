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

        function [vel_wave, ang_wave] = get_waves(obj)
            vel_wave = zeros(size(obj.vel_result_vec));
            ang_wave = zeros(size(obj.ang_result_vec));
            for i = 1:length(obj.brainwave_vec)
                wave_func = obj.brainwave_vec{i};
                [vel, ang] = wave_func(obj.mojave);
                vel_wave = vel_wave + vel;
                ang_wave = ang_wave + ang;
            end
        end

        function [vel_result, ang_result] = arbitrate_from_waves(obj, total_vel, total_ang)
            [~, Ivel] = max(total_vel);
            [~, Iang] = max(total_ang);
            vel_result = obj.vel_result_vec(Ivel(1));
            ang_result = obj.ang_result_vec(Iang(1));
        end

        function [vel_result, ang_result] = arbitrate(obj)
            [total_vel, total_ang] = obj.get_waves();
            [vel_result, ang_result] = obj.arbitrate_from_waves(total_vel, total_ang);
        end
    end 
end 