classdef robot < handle
    % ROBOT represents an instance of our Mojave rover

    properties
        arduino = arduino('COM9',"Nano3","Libraries",{'Servo','I2C'});
        lidar;
        robot_cam = webcam(1);
        camera_params = load("cameraParams.mat").cameraParams;
        ccm = zeros(4, 3);
        ir_vec = [{"A1"}, {"A2"}, {"A3"}, {"A6"}];
        sonar_vec = [{"A0"}, {"A7"}];
        lsm_obj
        neo

        steer_servo;
        last_angle = 0;
        throttle;
        last_speed = 0;
        pan_servo;
        cam_angle;
    end
    
    methods

        function obj = robot()
            obj.steer_servo = servo(obj.arduino, 'D5', 'MinPulseDuration', 0.9*10^-3, 'MaxPulseDuration', 2.1*10^-3);
            obj.steer(0);
            
            obj.throttle = servo(obj.arduino, 'D4', 'MinPulseDuration', 10*10^-6, 'MaxPulseDuration', 1925*10^-6);
            writePosition(obj.throttle, 0.5);
            addpath ../lidar
            addpath lidar
            obj.lidar = robo_lidar(obj.arduino);
            obj.lidar.tilt_lidar(0);
            
            obj.pan_servo = servo(obj.arduino, 'D7', 'MinPulseDuration', 10*10^-6, 'MaxPulseDuration', 2500*10^-6);
            obj.pan_camera(0);
            obj.cam_angle = 0;
        end

        function setup(obj)
            %obj.setup_lidar();
            %obj.lsm_obj = lsm9ds1(obj.arduino,"Bus", 1); % IMU magnetometer
            %obj.neo = NEO_M8U(obj.arduino); % GPS 
            for ir_pin = obj.ir_vec
                configurePin(obj.arduino, ir_pin{1}, 'AnalogInput')
            end
            for sonar_pin = obj.sonar_vec
                configurePin(obj.arduino, sonar_pin{1}, 'AnalogInput');
            end 
            disp('Setup complete');
        end 

        function distance = read_sonar(obj,voltage)
            % returns distance from voltage
            % function / transfer equation is derived from calibration curve
            a = 0.00002336;
            b = 2283; 
            c = 0.04631;
            distance = ((voltage-c)./a).^(1000./b);
        end

        function range_data_ir = ir_scan(obj)
            full_data = zeros(10, length(obj.ir_vec));
            for pin = 1:length(obj.ir_vec)
                for i = 1:10
                    full_data(i, pin) = readVoltage(obj.arduino,obj.ir_vec{pin});
                end
            end
            range_data_ir = real(obj.read_sharp(median(full_data)));

            try
                fig = evalin("base", "ir_fig");
                figure(fig);
                clf;
                plot(range_data_ir, 'ob');
                ylim([0 10]);
                xlim([0, 5]);
                gird on
            catch
            end
        end 

        function range_data_sonar = sonar_scan(obj)
            ranges = zeros(20, length(obj.sonar_vec));
            for pin = 1
                for i = 1:20
                    ranges(i, pin) = readVoltage(obj.arduino,obj.sonar_vec{pin});
                end
            end
            nonzero = ranges(ranges > 0);
            if isempty(nonzero)
                range_data_sonar = 0;
                return
            end
            range_data_sonar = median(nonzero);

            try
                fig = evalin("base", "sonar_fig");
                figure(fig);
                plot(range_data_sonar, 'ob');
                xlim([0 2]);
                ylim([0 10]);
            catch
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

        function distance = read_sharp(obj, voltage)
            % returns distance from voltage
            % power function / transfer equation is derived from calibration curve
            a = 15.56; % 15.04;  
            b = -0.7572; % -0.775; 
            c = -4.101; % -3.628; 
            distance = a.*(voltage.^b) + c;
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

        function setup_USB_camera(obj)
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

        function saw_chart = calculate_ccm(obj)
            % Calibrates the color correction matrix
            ccm_image = snapshot(obj.robot_cam);
            try
                obj.ccm = color_correction_matrix(ccm_image);
                saw_chart = true;
            catch
                saw_chart = false;
            end
        end

        function robot_image = sense_cam(obj)
            % This function acquires a single image from the USBCamera testCam
            % and displays it in a stand alone figure 
        
            robot_image = snapshot(obj.robot_cam);
            if obj.ccm ~= zeros(4, 3)
                robot_image = apply_color_correction(robot_image, obj.ccm);
            end
        end

        function [april_tags] = find_april_tags(obj)
            % find_april_tags finds and returns all april tags in the
            % camera's current view.

            img = obj.sense_cam();
            addpath april_tags
            tags_cam_frame = find_april_tags(img, obj.camera_params.Intrinsics);

            if isempty(tags_cam_frame)
                april_tags = [];
                return
            end

            R = eye(4);
            R(1:3, 1:3) = roty(obj.cam_angle);
            to_robot_frame = @(tag) april_tag(tag.id, tag.corners, rigid3d(tag.pose.T * R));

            april_tags = [to_robot_frame(tags_cam_frame(1))];
            for i = 2:length(tags_cam_frame)
                april_tags(i) = to_robot_frame(tags_cam_frame(2));
            end
        end

        function steer(obj,ang)
            l = 0.5;
            u = 0;
            inmin = 0;
            inmax = 30;
            pos = l + (ang-inmin)/(inmax-inmin)*(u-l);
            writePosition(obj.steer_servo, pos);
            obj.last_angle = ang;
        end 

        function set_speed(obj, speed)
            if (speed <= -1)
                writePosition(obj.throttle, 0.5);
            elseif (speed <= 0)
                writePosition(obj.throttle, 0);
            else
                pos = rescale(speed, 0.54, 0.6, 'InputMax', 1, 'InputMin', 0);
                writePosition(obj.throttle, pos);
            end
            obj.last_speed = speed;
        end

        function pan_camera(obj,ang)
            if (ang < -150)
                ang = -150;
            elseif (ang > 45)
                ang = 45;
            end
            l = 0.25;
            u = 0.64;
            inmin = 0;
            inmax = 90;
            pos = l + (ang-inmin)/(inmax-inmin)*(u-l);
            writePosition(obj.pan_servo, pos);
            obj.cam_angle = ang;
        end
    end 
end 