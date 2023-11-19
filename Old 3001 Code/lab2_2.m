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
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  model = Model(robot);
  
%  model.move_and_plot_arm([21.84	22.78	0.35], 4000);
%   model.move_and_plot_arm([-45, 45, 0], 4000);
%   joint_val = robot.measured_js(1, 0);
  robot.servo_jp([0 0 0]);
%   pause
%   transform_matrix = robot.fk3001(joint_val(1,:))
%   transform_matrix(6,1) = joint_val(1,1);
%   transform_matrix(6,2) = joint_val(1,2);
%   transform_matrix(6,3) = joint_val(1,3);
%   transform_matrix
%   csvwrite("lab2_section2_transformation_matrix_arb2.csv", transform_matrix)

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()
