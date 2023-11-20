%%
% RBE3001 - Laboratory 1 
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

  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
  jps = zeros(200, 3);
  tss = zeros(200, 1);

  robot.servo_jp([0, 0, 0]);

  pause(1);
  
  robot.servo_jp([-2.16	53.98	-14.05]);
    
  i = 0;
  tic
  while toc < 0.500
    i = i + 1;
    jp = robot.measured_js(1,0);
    jps(i,:) = jp(1,:);
    tss(i,:) = toc;
  end

  csvwrite("arb-pos-2-ni.csv", [jps, tss])
  
  hold on
  subplot(3,1,1);
  plot(tss,jps(:,1))
  title("J1 Position")
  xlabel("Time")
  ylabel("Degrees")

  subplot(3,1,2)
  plot(tss,jps(:,2))
  title("J2 Position") 
  xlabel("Time")
  ylabel("Degrees")

  subplot(3,1,3)
  plot(tss,jps(:,3))
  title("J3 Position")
  xlabel("Time")
  ylabel("Degrees")

  figure()

  hold off
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()