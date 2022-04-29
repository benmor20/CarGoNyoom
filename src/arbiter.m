%Must clear serial link
classdef arbiter
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        brainwave_vec
        vel_result_vec
        ang_result_vec
    end

    methods
        function obj = arbiter(brainwave_vec, vel_result_vec, ang_result_vec)
            obj.brainwave_vec = brainwave_vec;
            obj.vel_result_vec = vel_result_vec;
            obj.ang_result_vec = ang_result_vec;
        end

        function [vel_result, ang_result] = arbitrate(obj)
            total_vel = zeros(size(obj.brainwave_vec));
            total_ang = zeros(size(obj.brainwave_vec));
            for wave_func = obj.brainwave_vec
                [vel, ang] = wave_func();
                total_vel = total_vel + vel;
                total_ang = total_ang + ang;
            end
            [~, Ivel] = max(total_vel);
            [~, Iang] = max(total_ang);
            vel_result = obj.vel_result_vec(Ivel);
            ang_result = obj.ang_result_vec(Iang);
        end
    end 
end 