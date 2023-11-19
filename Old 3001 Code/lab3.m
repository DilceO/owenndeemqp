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
model = Model(robot);
recorder = Recorder(robot, "x.csv");
try
    % recorder.move_through_while_log_ik([60 -90 25; 75 0 230; 100 100 100;], [2000 2000 2000]);
    disp("ik3001([100 0 195]) = " + mat2str(robot.ik3001([100 0 195])))
    disp("ik3001([60 -90 25]) = " + mat2str(robot.ik3001([60 -90 25])))
    disp("ik3001([75 0 230]) = " + mat2str(robot.ik3001([75 0 230])))
    disp("ik3001([100 100 100]) = " + mat2str(robot.ik3001([100 100 100])))
    disp("")
    % robot.ik3001([100 0 195])
    % robot.ik3001(robot.fk3001([0 0 0]))
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()
