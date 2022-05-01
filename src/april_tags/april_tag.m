classdef april_tag
    % Represents a detected April Tag
    % Contains information for the id, the pixel coordinates of the
    % corners, and the pose as a rigid3d object.
    
    properties
        id
        corners
        pose
    end
    
    methods
        function obj = april_tag(id, corners, pose)
            % Create an april tag object
            obj.id = id;
            obj.corners = corners;
            obj.pose = pose;
        end
    end
end

