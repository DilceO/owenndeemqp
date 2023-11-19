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
try
  % Record Triangle Positions
  disp("Waiting for JP1:")
  pause
  jp1 = robot.measured_js(1, 0);
  jp1 = jp1(1, :);
  disp(jp1)

  disp("Waiting for JP2:")
  pause
  jp2 = robot.measured_js(1, 0);
  jp2 = jp2(1, :);
  disp(jp2)

  disp("Waiting for JP3:")
  pause
  jp3 = robot.measured_js(1, 0);
  jp3 = jp3(1, :);
  disp(jp3)

  robot.servo_jp(jp3)

  display("Waiting to start... (TURN MOTORS ON!)")
  pause



  recorder = Recorder(robot);
  recorder.move_through_while_log([jp1; jp2; jp3], [2000.0; 2000.0; 2000.0]);
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()
