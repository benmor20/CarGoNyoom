classdef robo_lidar < handle 

    properties
        lidar = serial('COM10','Baudrate',115200);
        tilt_servo;
    end 

    methods

        function obj = robo_lidar(arduino) 
            obj.tilt_servo = servo(arduino, 'D2', 'MinPulseDuration', 10*10^-6, 'MaxPulseDuration', 1925*10^-6);
        end 

    function lidar_setup(obj)
            set(obj.lidar, "Timeout", 0.1);
            set(obj.lidar, "InputBufferSize", 20000);
            set(obj.lidar, "Terminator", "LF/CR"); % initially was LF/CR
            % write a set of Hokuyo parameters to obj.lidar to configure it
            fopen(obj.lidar);
            pause(0.3);
            fprintf(obj.lidar, "SCIP2.0");
            pause(0.3);
            fscanf(obj.lidar);
            fprintf(obj.lidar, "VV");
            pause(0.3);
            fscanf(obj.lidar);
            fprintf(obj.lidar, "BM");
            pause(0.3);
            fscanf(obj.lidar);
            fprintf(obj.lidar, "MD0044072500");
            pause(0.3);
            fscanf(obj.lidar);
    end 


        function lidar_scan_continuous(obj)
           % initialize setup 
            % figure creates a stand-alone figure window, The resulting figure is the
            % current figure for all plots until you change it. 
            % create a primative line object to use plotting lidar data
            % XData and YData
            %disp('Read and Plot Lidar Data, type and hold ctrl-c to stop')
            theta = linspace(-135, 135-270/682, 682)*pi/180; % Convert Sensor steps to angles for plotting 
            %theta = theta(541:666);
            tStart = tic;                                       % start experiment timer
            iscan = 1;
            while (iscan == 1)                                   % continuous loop, type and hold cntl-c to break
                arm_length = 39; % mm
                min_tilt_angle = -20;
                max_tilt_angle = 10;
                increment = 2;
                phid = 0;
                obj.tilt_lidar(phid)
                %obj.tilt_lidar(phid);
                pause(0.4);
                [A] = FunRoboLidarScan(obj.lidar);              % actual lidar scan range data sored in [A]
                r = A(A < 20000);
                %r = ones(size(r))*30;
                %r = r(541:666);
                angs = theta(A < 20000);
                phi = deg2rad(phid);
                %lidar_step = linspace(0,270/180*pi,100); % 0-270 degrees in radians
                %angs = theta; %- (135*pi/180); % subtracted 135 degrees in radians to make center of scan 0: range is -135 to 135 degrees in radians
                [x_lidar_scan,y_lidar_scan] = pol2cart(angs,r); % converting polar coordinates to cartesian to make vector math easier
                x_to_base = x_lidar_scan + arm_length; % x direction distance from point to base 
                base_frame_vecs = [x_to_base;y_lidar_scan;zeros(1,length(x_to_base))];
                rotation_matrix_y = [cos(phi) 0 sin(phi); 0 1 0; -sin(phi) 0 cos(phi)];
                robot_frame_vecs = rotation_matrix_y * base_frame_vecs;
                plot(robot_frame_vecs(1,:),robot_frame_vecs(2,:),'.');
                %quiver3(0,0,0,arm_length*cos(-phi),0,arm_length*sin(-phi),'off')
                %hold on
%                     for i = 1:length(r)
%                         if x(i) < 2500 && y(i) < 2500 && z(i) < 2500
%                             plot3(x(i),z(i),y(i),'*','MarkerSize',1) 
%                             hold on
%                         end 
%                     end 
            %hold off
            axis equal
            xlim([-5000 5000])
            ylim([-5000 5000])
            xlabel('x')
            ylabel('y')
            zlabel('z')
            %clf;
            end 
        end 

        function single_scan = lidar_scan_single(obj)
           % initialize setup 
            % figure creates a stand-alone figure window, The resulting figure is the
            % current figure for all plots until you change it. 
            % create a primative line object to use plotting lidar data
            % XData and YData
            %disp('Read and Plot Lidar Data, type and hold ctrl-c to stop')
            theta = linspace(-135, 135-270/682, 682)*pi/180; % Convert Sensor steps to angles for plotting 
            
            phid = 0;
            arm_length = 39;    %mm
            obj.tilt_lidar(phid)
            [A] = FunRoboLidarScan(obj.lidar);              % actual lidar scan range data sored in [A]
%             within_range = ((A > 20) .* (A < 20000)) == 1;
            within_range = A > 40;
            r = A(within_range);
            angs = theta(within_range);
            phi = deg2rad(phid);
            [x_lidar_scan,y_lidar_scan] = pol2cart(angs,r); % converting polar coordinates to cartesian to make vector math easier
            x_to_base = x_lidar_scan + arm_length; % x direction distance from point to base 
            base_frame_vecs = [x_to_base;y_lidar_scan];
            rotation_matrix_y = [cos(phi) sin(phi); -sin(phi) cos(phi)];
            robot_frame_vecs = rotation_matrix_y * base_frame_vecs;
            single_scan = lidarScan(robot_frame_vecs');
        end 


        function point_cloud = lidar_scan_3d(obj)
            % initialize setup 
            % figure creates a stand-alone figure window, The resulting figure is the
            % current figure for all plots until you change it. 
            % create a primative line object to use plotting lidar data
            % XData and YData
            %disp('Read and Plot Lidar Data, type and hold ctrl-c to stop')
            theta = linspace(-135, 135-270/682, 682)*pi/180; % Convert Sensor steps to angles for plotting                 
            arm_length = 39; % mm
            min_tilt_angle = -20;
            max_tilt_angle = 10;
            increment = 2;
            point_cloud = pointCloud([0 0 0]);

            for phid = min_tilt_angle:increment:max_tilt_angle
                obj.tilt_lidar(phid);
                pause(0.4);
                [A] = FunRoboLidarScan(obj.lidar);              % actual lidar scan range data sored in [A]
                within_range = A < 20000;
                r = A(within_range);
                angs = theta(within_range);
                phi = deg2rad(phid);
                %lidar_step = linspace(0,270/180*pi,100); % 0-270 degrees in radians
                %angs = theta; %- (135*pi/180); % subtracted 135 degrees in radians to make center of scan 0: range is -135 to 135 degrees in radians
                [x_lidar_scan,y_lidar_scan] = pol2cart(angs,r); % converting polar coordinates to cartesian to make vector math easier
                x_to_base = x_lidar_scan + arm_length; % x direction distance from point to base 
                base_frame_vecs = [x_to_base;y_lidar_scan;zeros(1,length(x_to_base))];
                rotation_matrix_y = [cos(phi) 0 sin(phi); 0 1 0; -sin(phi) 0 cos(phi)];
                robot_frame_vecs = rotation_matrix_y * base_frame_vecs;
                new_cloud = pointCloud(robot_frame_vecs');

                if point_cloud.Count == 1
                    point_cloud = new_cloud;
                else
                    point_cloud = pccat([point_cloud, new_cloud]);
                end
            end 
        end
% 
%                 distance_to_object = vecnorm([laserRange.XData; laserRange.YData]);
%                 hole_threshold = 500;
%                 maxHoleLen = 0;
%                 maxStartAngle = 0; 
%                 maxEndAngle = 0; 
%                 
%                 drawnow                                         % will draw laserRange in open Laser Scan window
%                 pause(2)                                        % pause to allow serial communcations to keep up
%                 tElapsed = toc(tStart);                         % measure time (in sec) since start of experiment
%                 if(tElapsed > 600)                              % is experiment goes too long stop sample loop
%                     iscan = 0;
%                 end
%                 ang_step = 1;
%                 max_range =  1000; % mm
%                 distance_threshold = 200; % mm
%                 while ang_step < length(angles)
%                     if distance_to_object(ang_step) < distance_threshold && distance_to_object(ang_step) > 10
%                         x = laserRange.XData;
%                         y = laserRange.YData;
%                         len = get_length_closest(obj,ang_step,distance_to_object,angles,x,y);
%                         disp("There is an obstacle " + len + " mm long " + distance_to_object(ang_step) + "mm away.");
%                         ang_step = ang_step + len;
%                     end
%                     ang_step = ang_step + 1;
%                 end  
%             end 

        function equation_of_line = ransac(scan)
            equation_of_line = [];
        end 

        function [len,num_steps] = get_length_closest(obj,start_index,distance_to_object,angles,x,y)
        % distance is an array of distances corresponding to each point -
        % points are read left to right 
        stillTarget = true;          
        tolerance = 100;
        minDistance = 1000;
        targetIndex = start_index + 5;
        last_index = length(distance_to_object);
        blueTarget = 55; 
        yellowTarget = 47; 
        whiteTarget = 46;
        redTarget = 38;
        greenTarget = 31;
        missing_threshold = 3; % steps
        num_misses = 0;
        while targetIndex < length(distance_to_object) && stillTarget == true
            if targetIndex > 1
                current_target_distance = distance_to_object(targetIndex);
                target_difference = current_target_distance - distance_to_object(targetIndex - 1);
                if abs(target_difference) > tolerance && num_misses < missing_threshold
                    num_misses = num_misses + 1;
                elseif num_misses >= missing_threshold
                    last_index = targetIndex - 1;
                    stillTarget = false;
                end 
            else
                start_index = start_index + 1;
            end 
            if current_target_distance < minDistance
                minDistance = current_target_distance;
            end
            targetIndex = targetIndex + 1;
        end 
        startAngle = rad2deg(angles(start_index));
        endAngle = rad2deg(angles(last_index));
        points = [x(start_index),y(start_index);x(last_index),y(last_index)];
        len = sqrt((x(last_index)-x(start_index))^2+(y(last_index)-y(start_index))^2);
        num_steps = last_index - start_index;
        if len > 6
            disp(["Target found between angles ", startAngle, " & ", endAngle])
        end 
        end

        function tilt_lidar(obj,ang)
            if (ang < -20)
                ang = -20;
            elseif (ang > 60)
                ang = 60;
            end
            l = 0.25;
            u = 1;
            inmin = 0;
            inmax = 60;
            pos = l + (ang-inmin)/(inmax-inmin)*(u-l);
            writePosition(obj.tilt_servo, pos);
        end

        function lidar_shutdown(obj)
            fprintf(obj.lidar, 'QT');
            fclose(obj.lidar);
            clear obj;
        end
    end 
end 