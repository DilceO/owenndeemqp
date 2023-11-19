%%
% RBE3001 - Laboratory 2
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
robot = Robot(myHIDSimplePacketComs); 
tplanner = Traj_Planner();
recorder = Recorder(robot, "traj_triangles_jp.csv");
try
    %p1 = robot.ik3001([60, -90, 25]);
    %p2 = robot.ik3001([75, 0, 230]);
    %p3 = robot.ik3001([100, 100, 100]);
    p1 = [60, -90, 25];
    p2 = [75, 0, 230];
    p3 = [100, 100, 100];

    side1 = tplanner.quintic_traj(p1, p2, [0 0 0], [0 0 0], [0 0 0], [0 0 0], 3);
    side2 = tplanner.quintic_traj(p2, p3, [0 0 0], [0 0 0], [0 0 0], [0 0 0], 3);
    side3 = tplanner.quintic_traj(p3, p1, [0 0 0], [0 0 0], [0 0 0], [0 0 0], 3);

    entries1 = recorder.traj_while_log(side1, 3);
    entries2 = recorder.traj_while_log(side2, 3);
    entries3 = recorder.traj_while_log(side3, 3);
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()
