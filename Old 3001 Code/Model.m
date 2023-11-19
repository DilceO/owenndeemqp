classdef Model
    properties
        plot
        %% Axis
        % First Joint
        q11
        q12
        q13
        % Second Joint
        q21
        q22
        q23
        % Third joint
        q31
        q32
        q33
        % Velocity Arrows
        v1

        % EE Position
        ee_xyz
    end
    methods
        function self = Model()
            hold on
            self.plot = plot3(0, 0, 0);
            % First joint axis
            quiver3(0, 0, 0, 1, 0, 0, 10, 'red');
            quiver3(0, 0, 0, 0, 1, 0, 10, 'green');
            quiver3(0, 0, 0, 0, 0, 1, 10, 'blue');
            % Initializes quiver axis functions
            self.q11 = quiver3(0, 0, 0, 1, 0, 0, 10, 'red');
            self.q12 = quiver3(0, 0, 0, 0, 1, 0, 10, 'green');
            self.q13 = quiver3(0, 0, 0, 0, 0, 1, 10, 'blue');

            self.q21 = quiver3(0, 0, 0, 1, 0, 0, 10, 'red');
            self.q22 = quiver3(0, 0, 0, 0, 1, 0, 10, 'green');
            self.q23 = quiver3(0, 0, 0, 0, 0, 1, 10, 'blue');

            self.q31 = quiver3(0, 0, 0, 1, 0, 0, 10, 'red');
            self.q32 = quiver3(0, 0, 0, 0, 1, 0, 10, 'green');
            self.q33 = quiver3(0, 0, 0, 0, 0, 1, 10, 'blue');

            self.v1  = quiver3(0, 0, 0, 1, 0, 0, 10, 'black');

            self.ee_xyz = [0 0 0];
            hold off
            % Graph scaling and labeling
            axis equal
            axis([-150 150 -150 150 0 300])
            legend("Arm", "X", "Y", "Z")

            title('Robot Arm Visualization')

            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Z Axis');
        end
    
        function plot_arm(self, robot, q, ~)
            l1 = 95;    % Given link lengths
            l2 = 100;
            l3 = 100;
            % DH table values
            dh_table = [deg2rad(q(1, 1)),        l1, 0,  -pi/2;
                        -pi/2 + deg2rad(q(1, 2)), 0, l2, 0;
                        pi/2 + deg2rad(q(1, 3)), 0, l3, 0];
            ee_vel = robot.fdk3001(q(1, :), q(2, :));
            % Transformation matricies
            l1dh = robot.dh2mat(dh_table(1,:));
            l2dh = l1dh * robot.dh2mat(dh_table(2,:));
            l3dh = l2dh * robot.dh2mat(dh_table(3,:));
            % xyz coordinates of each joint
            xyz = [0 0 0; l1dh(1:3,4).'; l2dh(1:3, 4).'; l3dh(1:3, 4).'];
            self.ee_xyz = l1dh(1:3, 4).';
           
            set(self.plot, "XData", xyz(:,1), "YData", xyz(:,2), "ZData", xyz(:,3));
            % Updates axis on first joint
            set(self.q11, "XData", l1dh(1, 4), "YData", l1dh(2, 4), "ZData", l1dh(3, 4), "UData", l1dh(1, 1), "VData", l1dh(2, 1), "WData", l1dh(3, 1));
            set(self.q12, "XData", l1dh(1, 4), "YData", l1dh(2, 4), "ZData", l1dh(3, 4), "UData", l1dh(1, 2), "VData", l1dh(2, 2), "WData", l1dh(3, 2));
            set(self.q13, "XData", l1dh(1, 4), "YData", l1dh(2, 4), "ZData", l1dh(3, 4), "UData", l1dh(1, 3), "VData", l1dh(2, 3), "WData", l1dh(3, 3));
            % Updates axis on second joint
            set(self.q21, "XData", l2dh(1, 4), "YData", l2dh(2, 4), "ZData", l2dh(3, 4), "UData", l2dh(1, 1), "VData", l2dh(2, 1), "WData", l2dh(3, 1));
            set(self.q22, "XData", l2dh(1, 4), "YData", l2dh(2, 4), "ZData", l2dh(3, 4), "UData", l2dh(1, 2), "VData", l2dh(2, 2), "WData", l2dh(3, 2));
            set(self.q23, "XData", l2dh(1, 4), "YData", l2dh(2, 4), "ZData", l2dh(3, 4), "UData", l2dh(1, 3), "VData", l2dh(2, 3), "WData", l2dh(3, 3));
            % Updates axis on third joint
            set(self.q31, "XData", l3dh(1, 4), "YData", l3dh(2, 4), "ZData", l3dh(3, 4), "UData", l3dh(1, 1), "VData", l3dh(2, 1), "WData", l3dh(3, 1));
            set(self.q32, "XData", l3dh(1, 4), "YData", l3dh(2, 4), "ZData", l3dh(3, 4), "UData", l3dh(1, 2), "VData", l3dh(2, 2), "WData", l3dh(3, 2));
            set(self.q33, "XData", l3dh(1, 4), "YData", l3dh(2, 4), "ZData", l3dh(3, 4), "UData", l3dh(1, 3), "VData", l3dh(2, 3), "WData", l3dh(3, 3));
            % Velocity viz
            set(self.v1, "XData", l3dh(1, 4), "YData", l3dh(2, 4), "ZData", l3dh(3, 4), "UData", ee_vel(1), "VData", ee_vel(2), "WData", ee_vel(3));
        end

        function disp_on_graph(self, msg)
            % Updates graph is singularity is detected
            text(self.ee_xyz(1), self.ee_xyz(2), self.ee_xyz(3), msg, "Color", "Red");
        end
    end
end