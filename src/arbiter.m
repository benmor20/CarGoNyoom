%Must clear serial link
classdef arbiter
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        brainwave_vec
        result_vec
    end

    methods
        function obj = arbiter(brainwave_vec, result_vec)
            obj.brainwave_vec = brainwave_vec;
            obj.result_vec = result_vec;
        end

        function result = arbitrate(obj)
            total_wave = zeros(size(obj.brainwave_vec));
            for wave_func = obj.brainwave_vec
                total_wave = total_wave + wave_func();
            end
            [~, I] = max(total_wave);
            result = obj.result_vec(I);
        end
    end 
end 