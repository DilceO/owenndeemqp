classdef Recorder < handle
    properties
        robot
        filename
        data
        idx
        time
    end

    methods
        function self = Recorder(filename)
            self.filename = filename;
            self.data = zeros(100000, 13);
            self.idx = 1;
            self.time = -1;
        end

        function write(self)
            self.data = self.data(1:self.idx, :);
            table = array2table(self.data);
            table.Properties.VariableNames = [
                "Joint 1 Pos","Joint 2 Pos","Joint 3 Pos","Joint 1 Vel","Joint 2 Vel","Joint 3 Vel","EE Lin Vel X","EE Lin Vel Y","EE Lin Vel Z","EE Ang Vel X","EE Ang Vel Y","EE Ang Vel Z","Jacobian Det","Trajectory Time","Recorder Time"
            ];
            writetable(table, self.filename);
        end

        function self = add_datapoint(self, robot, q, time)
            if self.time == -1
                self.time = tic;
            end
            % Joint Position
            self.data(self.idx, 1:3) = q(1,:);
            % Joint Velocity
            self.data(self.idx, 4:6) = q(2,:);
            % Linear Velocity EE
            j = robot.jacob3001(q(1,:));
            jc = j * q(2,:).';
            self.data(self.idx, 7:9) = jc(1:3).';
            % Angular Velocity EE
            self.data(self.idx, 10:12) = jc(4:6).';
            % Det
            self.data(self.idx, 13) = det(j(1:3,:));
            % Local Time
            self.data(self.idx, 14) = time;
            % Global Time
            self.data(self.idx, 15)  = toc(self.time);

            self.idx = self.idx + 1;
        end
    end
end