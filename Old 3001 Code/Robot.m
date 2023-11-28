classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962;
        SERV_ID = 1848;
        SERVER_ID_READ = 1910;
        goal;
        move_callback;
        print_message_cb;
    end
    
    methods

        function set_move_callback(self, cb)
            self.move_callback = cb;
        end

        function set_print_callback(self, cb)
            self.print_message_cb = cb;
        end

        function use_model(self, model)
            self.move_callback = @model.plot_arm;
            self.print_message_cb = @model.disp_on_graph;
        end
        
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
	    %Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
            self.myHIDSimplePacketComs=dev; 
            self.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(self, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(self, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds,self.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(self, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(self.GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(intid, ds, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(self)
            self.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(self)
            self.writeGripper(0);
        end
        
        function interpolate_jp(self, xyz, time)
          self.goal = xyz;  % Sets goal for goal function

          packet = zeros(5, 1, 'single');
          packet(1) = time; %one second time
          packet(2) = 0; %linear interpolation
          packet(3) = xyz(1);   % Joint 1
          packet(4) = xyz(2);   % Joint 2
          packet(5) = xyz(3);   % Joint 3

          self.write(self.SERV_ID, packet); % Send Packet to Recieve Packet
          self.read(self.SERVER_ID_READ);   % Recieve Packet
        end

        function servo_jp(self, xyz)
          self.interpolate_jp(xyz, 0);      % Interpolates asap
        end

        function posVel = measured_js(self, GETPOS, GETVEL)
            posVel = zeros(2,3);        % Create Zeros Array
            
            if GETPOS == 1
                POS = 1910;
                pos = self.read(POS);   % Recieve Packet
                posVel(1,1) = pos(3);   % Joint 1
                posVel(1,2) = pos(5);   % Joint 2
                posVel(1,3) = pos(7);   % Joint 3
            end

            if GETVEL == 1
                VEL = 1822;
                vel = self.read(VEL);   % Recieve Packet
                posVel(2,1) = vel(3);   % Joint 1
                posVel(2,2) = vel(6);   % Joint 2
                posVel(2,3) = vel(9);   % Joint 3
            end
        end

        function position = setpoint_js(self)
            POS = 1910;

            self.write(POS, []);        % Send Packet to Recieve Packet
            posPacket = self.read(POS); % Recieve Packet

            position = zeros(1,3);          % Creates zero matrix

            position(1,1) = posPacket(2);   % Joint 1
            position(1,2) = posPacket(4);   % Joint 2
            position(1,3) = posPacket(6);   % Joint 3

        end

        function ret = goal_js(self)
            ret = self.goal;            % Pre-defined Goal
        end

        function transform = dh2mat(~, dhparams)
            theta = dhparams(1); % Extracts components
            d = dhparams(2);
            a = dhparams(3);
            alpha = dhparams(4);
            % DH transformation
            transform = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                         0,          sin(alpha),             cos(alpha),            d           ;
                         0,          0,                      0,                     1           ];
        end

        function retMat = dh2fk(self, dhTable)    
            rows = height(dhTable);                             % Finds number of rows
            retMat = [ 1 0 0 0;                                 % Creates identity matrix as base
                       0 1 0 0;
                       0 0 1 0;
                       0 0 0 1];
            for i = 1:rows                                      % loops for number of rows
                retMat = retMat * self.dh2mat(dhTable(i,:));    % multiplies each transformation matrix
            end
        end

        function tnsfm = fk3001(self, angles)
            l1 = 95;    % Given link lengths
            l2 = 100;
            l3 = 100;
            % DH table values
            dh_table = [deg2rad(angles(1)),        l1, 0,  -pi/2;
                       -pi/2 + deg2rad(angles(2)), 0,  l2, 0;
                        pi/2 + deg2rad(angles(3)), 0,  l3, 0];

            tnsfm = self.dh2fk(dh_table);           % Calculates transform from DH table
        end

        function tnsfm = measured_cp(self)          % Finds end effector position
            res = self.measured_js(1, 0);           
            tnsfm = self.fk3001(res(1,:));
        end

        function tnsfm = setpoint_cp(self)
            tnsfm = self.fk3001(self.setpoint_js);  % Finds transform for setpoint
        end

        function tnsfm = goal_cp(self)
            tnsfm = self.fk3001(self.goal_js);      % Finds transform for goal
        end

        function tnsfm = ik3001(~, ee)
                x = ee(1);                          % End-Effector positions
                y = ee(2);
                z1 = ee(3)-95;
            
                %z1 = z - 95;
            
%                 j3 = (pi/2)-acos((x^2 + y^2 + (z1)^2 - 2 * 100^2) / (-2 * 100^2));
%                 j3 = simplify(j3)
                j3 = pi/2 - acos(- x^2/20000 - y^2/20000 - z1^2/20000 + 1);
%                 j2 = (pi/2)-atan2(z1,sqrt(x^2 + y^2)) - asin((sin((pi/2)-j3) * 100)/sqrt(x^2 + y^2 + z1^2));
%                 j2 = simplify(j2)
                j2 = pi/2 - atan2(z1, (x^2 + y^2)^(1/2)) - asin((100*cos(j3))/(x^2 + y^2 + z1^2)^(1/2));
                j1 = atan2(y,x);

                tnsfm = [j1 * (180/pi), j2 * (180/pi), j3*(180/pi)];
        end
        
        function run_trajectory(self, coeffs, time, is_ts) 
            is_cubic = width(coeffs) == 4;                                  % Checks if cubic or quintic
            if is_cubic
                tic;
                while toc < time                                            % Runs cubic trajectory
                    t = toc;
                    sample = coeffs * [1; t; (t)^2; (t)^3];
                    if is_ts
                        sample = self.ik3001(sample);
                    end
                    self.servo_jp(sample);

                    q = self.measured_js(1, 1);

                    if ~self.check_safe(q)                                  % Check if close to singularity
                        self.print_message_cb("Arm approaching singularity, stopping!")
                        break
                    end

                    self.move_callback(self, q, t);
                    drawnow
                end
            else
                tic;
                while toc < time                                            % Runs cubic trajectory
                    t = toc;
                    sample = coeffs * [1; t; (t)^2; (t)^3; (t^4); (t)^5];
                    if is_ts
                        sample = self.ik3001(sample);
                    end
                    self.servo_jp(sample);

                    q = self.measured_js(1, 1);

                    if ~self.check_safe(q)                                  % Check if close to singularity
                        self.print_message_cb("Arm approaching singularity, stopping!");
                        break
                    end

                    self.move_callback(self, q, t);

                    drawnow
                end
            end
        end

        function j = jacob3001(~, q)
%             syms a1 a2 a3
            a1 = deg2rad(q(1)); % Coverts degrees to radians
            a2 = deg2rad(q(2));
            a3 = deg2rad(q(3));
%             j = [
%                 - 100*sin(a1)*cos(pi/2 - a2) - 100*cos(pi/2 + a3)*sin(a1)*cos(pi/2 - a2) - 100*sin(a1)*sin(pi/2 + a3)*sin(pi/2 - a2),   100*cos(a1)*sin(pi/2 - a2) + 100*cos(a1)*cos(pi/2 + a3)*sin(pi/2 - a2) - 100*cos(a1)*sin(pi/2 + a3)*cos(pi/2 - a2), 100*cos(a1)*cos(pi/2 + a3)*sin(pi/2 - a2) - 100*cos(a1)*sin(pi/2 + a3)*cos(pi/2 - a2);
%                 100*cos(a1)*cos(pi/2 - a2) + 100*cos(a1)*cos(pi/2 + a3)*cos(pi/2 - a2) + 100*cos(a1)*sin(pi/2 + a3)*sin(pi/2 - a2),     100*sin(a1)*sin(pi/2 - a2) + 100*cos(pi/2 + a3)*sin(a1)*sin(pi/2 - a2) - 100*sin(a1)*sin(pi/2 + a3)*cos(pi/2 - a2), 100*cos(pi/2 + a3)*sin(a1)*sin(pi/2 - a2) - 100*sin(a1)*sin(pi/2 + a3)*cos(pi/2 - a2);
%                 0,                                                                              S                                        - 100*cos(pi/2 - a2) - 100*sin(pi/2 + a3)*sin(pi/2 - a2) - 100*cos(pi/2 + a3)*cos(pi/2 - a2),                       - 100*sin(pi/2 + a3)*sin(pi/2 - a2) - 100*cos(pi/2 + a3)*cos(pi/2 - a2);
%                 0,                                                                                                                      -sin(a1),                                                                                                           -sin(a1);
%                 0,                                                                                                                      cos(a1),                                                                                                            cos(a1);
%                 1,                                                                                                                      0,                                                                                                                  0];
%             j = simply(j);
            % Jacobian Matrix
            j = [-100*sin(a1)*(cos(a2 + a3) + sin(a2)), -100*cos(a1)*(sin(a2 + a3) - cos(a2)), -100*sin(a2 + a3)*cos(a1);
                  100*cos(a1)*(cos(a2 + a3) + sin(a2)), -100*sin(a1)*(sin(a2 + a3) - cos(a2)), -100*sin(a2 + a3)*sin(a1);
                                                     0,      - 100*cos(a2 + a3) - 100*sin(a2),         -100*cos(a2 + a3);
                                                     0,                              -sin(a1),                  -sin(a1);
                                                     0,                               cos(a1),                   cos(a1);
                                                     1,                                     0,                         0];
        end

        function f = fdk3001(self, q, v)
            f = self.jacob3001(q) * v.';                                    % solves forward kinematics in velocity
        end

        function move_to_point(self, pt, v)                                 % moves to point using a given velocity
            epsilon = 10;                                                   % Tolerance in mm
            hz = 10;                                                        % refresh rate

            q = self.measured_js(1, 1);

            ee_pos = self.fk3001(q(1,:));
            ee_pos = ee_pos(1:3, 4).';                                      % Finds end effector position

            dist = norm(pt - ee_pos);
            
            time = tic;
            while dist > epsilon                                            % While distance is outside of the given tolerance
                if toc(time) > 1/hz
                    q = self.measured_js(1, 1);
                    
                    js = q(1, :);

                    ee_pos = self.fk3001(js);
                    ee_pos = ee_pos(1:3, 4).';
        
                    dist = norm(pt - ee_pos);                               % distance from end effector and target point
                    dir  = (pt - ee_pos) / dist;                            % direction from end effector to target point
                    
                    j = self.jacob3001(js);
                    j = inv(j(1:3, :));
        
                    jvs = (j * (dir.' .* v)) .* (180/pi);                   % Joint velosities
                    jps = (js + (jvs .* (1/hz)).');                         % Joint positions

                    self.interpolate_jp(jps, 1/hz * 1000);                  % move given distance over given time at a certain velocity
                    self.move_callback(self, q, 1/hz);

                    time = tic;
                end
            end
        end

        function safe = check_safe(self, q)                                 % Checks if getting close to singularity
            j = self.jacob3001(q(1,:));
            d = det(j(1:3,:));

            safe = (d > 100000);                                            % 100000 seems to be a reasonable value for the determinite limit
        end
    end
end
