classdef robot
    % ROBOT represents an instance of our Mojave rover

    properties
        camlist = webcamlist;
        %robot_cam = webcam(2);
        %arduino = arduino(arduino_comport, 'Uno', 'Libraries', 'Servo');
        %lidar = serial(lidar_comport,'baudrate',115200);
        %drive_motor = servo(arduino, 'D3', 'MinPulseDuration', 10*10^-6, 'MaxPulseDuration', 1925*10^-6);
        %steering_motor = servo(arduino, 'D5', 'MinPulseDuration', 10*10^-6, 'MaxPulseDuration', 1925*10^-6);
        %pan_servo = servo(arduino, 'D6', 'MinPulseDuration', 10*10^-6, 'MaxPulseDuration', 1925*10^-6);
        ir_vec = ['A2','A3','A4','A5','A6','A7'];
        sonar_vec = ['A0','A1'];
        lsm_obj
        neo
    end

    methods
       
        function obj = lidar_setup(obj)
            set(obj.lidar, "Timeout", 3);
            set(obj.lidar, "InputBufferSize", 20000);
            set(obj.lidar, "Terminator", "LF"); % initially was LF/CR
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
            disp('Read and Plot Lidar Data, type and hold ctrl-c to stop')
            angles = (-120:240/682:120-240/682)*pi/180; % Convert Sensor steps to angles for plotting 
            angles = angles(541:666);
            tStart = tic;                                       % start experiment timer
            iscan = 1;
            while(iscan == 1)                                    % continuous loop, type and hold cntl-c to break
                [A] = FunRoboLidarScan(obj.lidar);                  % actual lidar scan range data sored in [A]
                A = A(541:666);
                laserRange.XData = A.*cos(angles);              % Use trig to find x-coord of range
                laserRange.YData = A.*sin(angles);              % Use trig to find y-coord of rangematlab:matlab.internal.language.commandline.executeCode('cd ''C:\Users\busui\OneDrive - Olin College of Engineering\Desktop\FunRobo\STA Lab''')
                distance_to_object = vecnorm([laserRange.XData; laserRange.YData]);
                hole_threshold = 500; 
                maxHoleLen = 0;
                maxStartAngle = 0; 
                maxEndAngle = 0; 
                % drawnow -updates figures and processes any pending callbacks.
                % Use this command if you modify graphics objects and want to see the
                % updates on the screen immedicately. Use here to update laser range
                % draws
                drawnow                                         % will draw laserRange in open Laser Scan window
                pause(2)                                      % pause to allow serial communcations to keep up
                tElapsed = toc(tStart);                         % measure time (in sec) since start of experiment
                if(tElapsed > 10)                               % is experiment goes too long stop sample loop
                    iscan = 0;
                end
            end 
            ang = 1;
            while ang < 127
                if distance_to_object(1,ang) > hole_threshold
                    [len, startAngle, endAngle] = findHole(ang,distance_to_object,angles);
                    if len > maxHoleLen
                        maxHoleLen = len;
                        maxStartAngle = startAngle; 
                        maxEndAngle = endAngle;
                        ang = ang + len;
                    end
                else
                    %findTarget(ang,distance_to_object,angles);
                end 
                ang = ang + 1;
            end
            disp("Largest hole found between " + maxStartAngle + " degrees and " + maxEndAngle + " degrees.")
            disp('Lidar scan ended');
        end 

        function obj = setup(obj)
            obj.setup_lidar();
            obj.lsm_obj = lsm9ds1(obj.arduino,"Bus", 1); % IMU magnetometer
            obj.neo = NEO_M8U(obj.arduino); % GPS 
            for ir_pin = obj.ir_vec
                configurePin(obj.arduino,ir_pin, 'AnalogInput');
            end
            for sonar_pin = obj.sonar_vec
                configurePin(obj.arduino,sonar_pin, 'AnalogInput');
            end 
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

        function range_data = sense_ir(obj)
            range_data =  readVoltage(obj.arduino,obj.ir_vec);
        end 

        function range_data = sense_sonar(obj)
            range_data =  readVoltage(obj.arduino,obj.sonar_vec);
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

        function [robotImage] = sense_cam(obj)
            % This function acquires a single image from the USBCamera testCam
            % and displays it in a stand alone figure 
        
            robotImage = snapshot(obj.robot_cam);
        end

        function obj = lidar_shutdown(obj)
            fprintf(obj.lidar, 'QT');
            fclose(obj.lidar);
            clear obj;
        end
    end 
end 