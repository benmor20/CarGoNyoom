classdef robot
    % ROBOT represents an instance of our Mojave rover
    % TODO: 
    % Convert angle to position

    properties
        robot_cam = webcam(1);
        camera_params = load("cameraParams.mat").cameraParams;
        arduino = arduino
        lidar = serialport('COM5',115200);
        ir_vec = ["A1","A2","A3","A6"];
        sonar_vec = ["A0","A7"];
        lsm_obj
        neo
        steer_servo; % 0.37-0.66
        throttle;
        pan_servo;
        tilt_servo;
        %range_data_ir;
    end

    methods

        function obj = robot()
            obj.steer_servo = servo(obj.arduino, 'D5', 'MinPulseDuration', 10*10^-6, 'MaxPulseDuration', 1925*10^-6);
            obj.steer(0);
            obj.throttle = servo(obj.arduino, 'D6', 'MinPulseDuration', 10*10^-6, 'MaxPulseDuration', 1925*10^-6);
            writePosition(obj.throttle, 0.5);
            obj.tilt_servo = servo(obj.arduino, 'D2', 'MinPulseDuration', 10*10^-6, 'MaxPulseDuration', 1925*10^-6);
            obj.tilt_lidar(0);
            obj.pan_servo = servo(obj.arduino, 'D3', 'MinPulseDuration', 10*10^-6, 'MaxPulseDuration', 1925*10^-6);
        end
       
        function obj = lidar_setup(obj)
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

        function obj = initialize_lidar_display(obj)
            % figure creates a stand-alone figure window, The resulting figure is the
            % current figure for all plots until you change it. 
            LaserPlot1.figure = figure('Name','Hokuyo URG-04LX data','NumberTitle','off',...
                'MenuBar','figure','units','normalized','Visible','on')
            LaserPlot1.axis1=axes('parent',LaserPlot1.figure,'units','normalized','NextPlot','replace');
            grid(LaserPlot1.axis1,'on')
            LaserPlot1.axis1.Title.String = 'Laser Scans';
            LaserPlot1.XLabel.String = 'X Axis';
            LaserPlot1.YLabel.String = 'Y Axis';
            % create a primative line object to use plotting lidar data
            % XData and YData
            laserRange = line('Parent',LaserPlot1.axis1,'XData',[],'YData',[],'LineStyle','none',...
                'marker','.','color','b','LineWidth',2);
            grid on 
            range = 700;
            axis([-range range -range range])
            xlabel('x (mm)')
            ylabel('y (mm)')
            disp('Laser scan figure set');
        end 

        function obj = lidar_scan(obj)
            % initialize setup 
            % figure creates a stand-alone figure window, The resulting figure is the
            % current figure for all plots until you change it. 
            LaserPlot1.figure = figure('Name','Hokuyo URG-04LX data','NumberTitle','off',...
                'MenuBar','figure','units','normalized','Visible','on')
            LaserPlot1.axis1=axes('parent',LaserPlot1.figure,'units','normalized','NextPlot','replace');
            grid(LaserPlot1.axis1,'on')
            LaserPlot1.axis1.Title.String = 'Laser Scans';
            LaserPlot1.XLabel.String = 'X Axis';
            LaserPlot1.YLabel.String = 'Y Axis';
            % create a primative line object to use plotting lidar data
            % XData and YData
            laserRange = line('Parent',LaserPlot1.axis1,'XData',[],'YData',[],'LineStyle','none',...
                'marker','.','color','b','LineWidth',2);
            grid on 
            range = 700;
            axis([-range range -range range])
            xlabel('x (mm)')
            ylabel('y (mm)')
            disp('Laser scan figure set');

            %disp('Read and Plot Lidar Data, type and hold ctrl-c to stop')
            angles = (-120:240/682:120-240/682)*pi/180; % Convert Sensor steps to angles for plotting 
            %angles = angles(541:666);
            tStart = tic;                                       % start experiment timer
            iscan = 1;
            while(iscan == 1)                                   % continuous loop, type and hold cntl-c to break
                [A] = FunRoboLidarScan(obj.lidar);              % actual lidar scan range data sored in [A]
                %A = A(541:666);
                laserRange.XData = A.*cos(angles);              % Use trig to find x-coord of range
                laserRange.YData = A.*sin(angles);              % Use trig to find y-coord of rangematlab:matlab.internal.language.commandline.executeCode('cd ''C:\Users\busui\OneDrive - Olin College of Engineering\Desktop\FunRobo\STA Lab''')
                distance_to_object = vecnorm([laserRange.XData; laserRange.YData]);
                hole_threshold = 500;
                maxHoleLen = 0;
                maxStartAngle = 0; 
                maxEndAngle = 0; 
                
                drawnow                                         % will draw laserRange in open Laser Scan window
                pause(2)                                      % pause to allow serial communcations to keep up
                tElapsed = toc(tStart);                         % measure time (in sec) since start of experiment
                if(tElapsed > 600)                               % is experiment goes too long stop sample loop
                    iscan = 0;
                end
                ang_step = 1;
                max_range =  1000; % mm
                distance_threshold = 200; % mm
                while ang_step < length(angles)
                    if distance_to_object(ang_step) < distance_threshold && distance_to_object(ang_step) > 10
                        x = laserRange.XData;
                        y = laserRange.YData;
                        len = get_length_closest(obj,ang_step,distance_to_object,angles,x,y);
                        disp("There is an obstacle " + len + " mm long " + distance_to_object(ang_step) + "mm away.");
                        ang_step = ang_step + len;
                    end
                    ang_step = ang_step + 1;
                end  
            end 
        end 

        function obj = lidar_scan_3d(obj)
            % initialize setup 
            % figure creates a stand-alone figure window, The resulting figure is the
            % current figure for all plots until you change it. 
            LaserPlot1.figure = figure('Name','Hokuyo URG-04LX data','NumberTitle','off',...
                'MenuBar','figure','units','normalized','Visible','on')
            LaserPlot1.axis1=axes('parent',LaserPlot1.figure,'units','normalized','NextPlot','replace');
            grid(LaserPlot1.axis1,'on')
            LaserPlot1.axis1.Title.String = 'Laser Scans';
            LaserPlot1.XLabel.String = 'X Axis';
            LaserPlot1.YLabel.String = 'Y Axis';
            % create a primative line object to use plotting lidar data
            % XData and YData
            laserRange = line('Parent',LaserPlot1.axis1,'XData',[],'YData',[],'LineStyle','none',...
                'marker','.','color','b','LineWidth',2);
            grid on 
            range = 700;
            axis([-range range -range range])
            xlabel('x (mm)')
            ylabel('y (mm)')
            disp('Laser scan figure set');

            %disp('Read and Plot Lidar Data, type and hold ctrl-c to stop')
            ang = (-120:240/682:120-240/682)*pi/180; % Convert Sensor steps to angles for plotting 
            ang = ang(541:666);
            tStart = tic;                                       % start experiment timer
            iscan = 1;
            while (iscan == 1)                                   % continuous loop, type and hold cntl-c to break
                [A] = FunRoboLidarScan(obj.lidar);              % actual lidar scan range data sored in [A]
                A = A(541:666);
                laserRange.XData = A.*cos(ang);              % Use trig to find x-coord of range
                laserRange.YData = A.*sin(ang);              % Use trig to find y-coord of rangematlab:matlab.internal.language.commandline.executeCode('cd ''C:\Users\busui\OneDrive - Olin College of Engineering\Desktop\FunRobo\STA Lab''')
                [r,theta] = cart2pol(laserRange.XData,laserRange.YData);   
                %servo_angle_phi = 0;
                arm_length = 39; % mm
%                 x = arm_length.*sin(phi)+r.*cos(phi).*cos(theta); % x coordinate from base
%                 y = arm_length.*cos(phi)-r.*sin(phi).*cos(theta); % y coordinate from base
%                 z = r.*sin(theta); % z coordinate from base
%                 plot3(x,z,y,'*') % plot points in 3d (z is out of the page)
%                 xlabel('x')
%                 ylabel('z')
%                 zlabel('y')
                min_tilt_angle = 0;
                max_tilt_angle = 60;
                increment = 5;

                hold on;
                for phi = min_tilt_angle:increment:max_tilt_angle
                    obj.tilt_lidar(phi);
                    pause(0.2);
                    phi = deg2rad(phi);
                    x = arm_length.*sin(phi)+r.*cos(phi).*cos(theta); % x coordinate from base
                    y = arm_length.*cos(phi)-r.*sin(phi).*cos(theta); % y coordinate from base
                    z = r.*sin(theta); % z coordinate from base
                    plot3(x,z,y,'*') 
                    hold on
                end 
            end 
%                 end 
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
%         end 

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

        function obj = setup(obj)
            %obj.setup_lidar();
            %obj.lsm_obj = lsm9ds1(obj.arduino,"Bus", 1); % IMU magnetometer
            %obj.neo = NEO_M8U(obj.arduino); % GPS 
            for ir_pin = obj.ir_vec
                configurePin(obj.arduino,ir_pin, 'AnalogInput')
            end
            for sonar_pin = obj.sonar_vec
                configurePin(obj.arduino,sonar_pin, 'AnalogInput');
            end 
            disp('Setup complete');
        end 

        function position_data = get_distance_sonar(obj, sensor_index)
            num_tests = 20; % number of datapoints (raw range data) to take in at a given sensor 
            temp_range = []; % clear tempRange - stores 20 samples of voltage data from a particular sensor 
            for j = 1:numTests % loop till numtests data captured
                temp_range(j) = sense(obj.arduino,obj.sonar_vec(sensor_index)); % collect data from robot sensors and store in tempRange at index j
                pause(0.1); % pause for 0.1 sec
            end
            range_data = median(temp_range); % find median value between 20 raw voltage samples to filter the data. Median is not as easily influenced by outliers compared to avg. 
            position_data = read_sonar(range_data); % converts median voltage to distance in cm from sensor
        end 

        function distance = read_sonar(voltage)
            % returns distance from voltage
            % function / transfer equation is derived from calibration curve
            a = 0.00002336;
            b = 2283; 
            c = 0.04631;
            distance = ((voltage-c)/a)^(1000/b);
        end

        function range_data_ir = ir_scan(obj)
            for pin = 1:length(obj.ir_vec)
                range_data_ir(pin) = readVoltage(obj.arduino,obj.ir_vec(pin));
            end
        end 

        function range_data_sonar = sonar_scan(obj)
            for pin = 1:length(obj.sonar_vec)
                range_data_sonar(pin) = readVoltage(obj.arduino,obj.sonar_vec(pin));
            end
        end 

        function identify_target_sonar(sensor_index,position_data)
            angles = 0; % corresponding angles 
            threshold = 31; % threshold distance for hole in cm 
            if position_ata < threshold % m - if position is close enough to be considered a target
                disp("Target found " + position_data + "cm away at a " + angles(sensor_index) + " degree angle. ID pending."); % it the sensor is not the middle one, A2, and distance is within range for an object, detect object but don't identify it. 
            else
                disp("Hole found at " + angles(sensor_index) + " degrees"); % if none of the other criteria are met, then it means there is a hole in front of the A2 sensor, which is the only case-scenario in which this line runs
            end 
        end 

        function distance = read_sharp(voltage)
            % returns distance from voltage
            % power function / transfer equation is derived from calibration curve
            a = 15.56; % 15.04;  
            b = -0.7572; % -0.775; 
            c = -4.101; % -3.628; 
            distance = a*(voltage^b) + c;
        end

        function identify_target_ir(sensor_index,position_data)
            angles = [-25 0 25]; % corresponding angles 
            whiteTarget = 7.7; % distance threshold for white target in cm
            greenTarget = 7.3; % distance threshold for green target in cm 
            yellowUpsideDown = 7; % distance threshold for yellow target in upside-down orientation in cm 
            redTarget = 6.5; % distance threshold for red target in cm
            yellowUpright = 6; % distance threshold for yellow target - upright orientation in cm 
            threshold = 10; % threshold distance for hole in cm 
        
            if position_data < threshold % m - if position is close enough to be considered a target
                if sensor_index == 2 % if it's the middle sensor, in port A2
                    % place sensor at 10 cm
                    if position_data >= whiteTarget % if distance is between the minimum for white target and the max for an object
                        disp("White target found " + position_data + "cm away at a " + angles(sensor_index) + " degree angle."); % it's white 
                    elseif position_data > greenTarget % if distance is between min for green and min for white 
                        disp("Green target found " + position_data + "cm away at a " + angles(sensor_index) + " degree angle."); % it's green
                    elseif position_data > yellowUpsideDown % if distance is between min distance for yellow and min for green 
                        disp("Yellow target found " + position_data + "cm away at a " + angles(sensor_index) + " degree angle."); % it's yellow 
                    elseif position_data > redTarget % if distance is between min dist for red and min for yellow
                        disp("Red target found " + position_data + "cm away at a " + angles(sensor_index) + " degree angle."); % it's red
                    elseif position_data > yellowUpright % if dist is between min dist for yellow - upright and min for red 
                        disp("Yellow target found " + position_data + "cm away at a " + angles(sensor_index) + " degree angle."); % it's yellow upright
                    else       
                        disp("Blue target found " + position_data + "cm away at a " + angles(sensor_index) + " degree angle."); % if the dist is shorter than the minimum for yellow - upright, it's blue
                    end
                else 
                    disp("Target found " + position_data + "cm away at a " + angles(sensor_index) + " degree angle. ID pending."); % it the sensor is not the middle one, A2, and distance is within range for an object, detect object but don't identify it. 
                end 
        
            elseif sensor_index ~= 2 % if sensor is one of the outer ones (A1 or A3), and distance is greater than initial 10cm threshold for center sensor
                if position_data < 15 % if one of the outer sensors detects the target at a further distance away 
                    disp("Target found " + position_data + "cm away at a " + angles(sensor_index) + " degree angle. ID pending.") % display that a target has been found 
                else 
                    disp("Hole found at " + angles(sensor_index) + " degrees"); % if the distance is greater than 15cm, display that a hole has been found. 
                end 
            else
                disp("Hole found at " + angles(sensor_index) + " degrees"); % if none of the other criteria are met, then it means there is a hole in front of the A2 sensor, which is the only case-scenario in which this line runs
            end 
        end 

        function [] = setup_USB_camera(obj)
            % SETUPUSBCAMERA creates and configures a Webcam to be a simple robot
            % vision system. It requires a standard Webcam attached to
            % your computer and takes webcam object name as sole input. 
            % You ned to set your cameras unique parameters to optimize picture 
            % J. Brown 2022 Rev 1
        
            % Fix auto exposure set it to manual, set whitebalance to manual too
            obj.robot_cam.ExposureMode = 'manual'; % for LifeCam
            obj.robot_cam.WhiteBalanceMode = 'manual';
            % experimentlaly determine best exposure setting for lab, enter here
            %obj.robot_cam.Exposure = ; % for LifeCam
            % robotCam.WhiteBalanceMode = 'manual'; % for LifeCam
            %obj.robot_cam.Brightness = 64;
        end 

        function [robot_image] = sense_cam(obj)
            % This function acquires a single image from the USBCamera testCam
            % and displays it in a stand alone figure 
        
            robot_image = snapshot(obj.robot_cam);
        end

        function [april_tags] = find_april_tags(obj)
            % find_april_tags finds and returns all april tags in the
            % camera's current view.

            img = obj.sense_cam();
            [ids, corners, poses] = readAprilTag(img, 'tag36h11', obj.camera_params.Intrinsics, 164);
            if isempty(ids)
                april_tags = [];
            else
                april_tags = [april_tag(ids(1), corners(:, :, 1), poses(1))];
                for i = 2:length(ids)
                    april_tags(i) = april_tag(ids(i), corners(:, :, i), poses(i));
                end
            end
        end

        function obj = steer(obj,ang)
            pos = rescale(ang, 0.37, 0.66, 'InputMin', -30, 'InputMax', 30);
            writePosition(obj.steer_servo, pos);
        end 

        function obj = turn_throttle(obj,motor,speed,duration)
            t_start = tic;
            control_flag = 1;  
            while control_flag == 1                 % measure time (in sec) since start of experiment
                tElapsed = toc(t_start); 
                if(tElapsed > duration)                               % is experiment goes too long stop sample loop
                    control_flag = 0;
                end 
            end 
        end 

        function set_speed(obj, speed)
            if (speed < 0)
                speed = 0;
            elseif (speed > 1)
                speed = 1;
            end
            pos = rescale(speed, 0.5, 0.6, 'InputMax', 1, 'InputMin', 0);
            writePosition(obj.throttle, pos);
        end

        function obj = tilt_lidar(obj,ang)
            if (ang < -20)
                ang = -20;
            elseif (ang > 90)   % Technically can go a bit further but whatev
                ang = 90;
            end
            l = 0.41;
            u = 0.95;
            inmin = 0;
            inmax = 90;
            pos = l + (ang-inmin)/(inmax-inmin)*(u-l);
            writePosition(obj.tilt_servo, pos);
        end

        function obj = lidar_shutdown(obj)
            fprintf(obj.lidar, 'QT');
            fclose(obj.lidar);
            clear obj;
        end
    end 
end 