%%
% RBE 3001 Lab 5 example code!
%%
clc;
clear;
clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end


javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);
robot.set_move_callback(@empty_cb);
robot.set_print_callback(@sing_cb);
tplanner =  Traj_Planner();

robot.servo_jp([0 0 0]);

%% What to Run if new time
%     cam = Camera
%     save("camParams.mat", "cam")
%%

% try
%     cam;
%     disp("Cam Already Set Up")
% catch exception
%     disp("Have to do some setup...")
%     cam = Camera();
%     save("camParams.mat","cam");
% end


try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
    cam = Camera();
    save("camParams.mat","cam");
    disp("Saved Camera Parameters to camParams.mat");
end

rob2grid = [
    0 1  0 75;
    1 0  0 -100;
    0 0 -1  0;
    0 0 0 1
    ];

%% Main Loop

try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    disp("Make sure board is clear. Press a key when ready")
    pause();
    cam.calculateCameraPos();
    disp("Add objects. Press a key when ready")
    pause();
    

    %% red_mask
    curr_task = "Image Processing";
    ball_pos_imag = [0 0];
    pt = [ 0 0 0 ];

    locs = [];
    color = [""];

    while(1)

        switch curr_task
            
            case "Image Processing"
                robot.openGripper();
                I = cam.getImage();
               
                i = 1;
    
                colors = ["red", "yellow", "orange", "green", "blue"];
                for c = colors
                    loc = find_Obj(I, c);
                    if height(loc) ~= 0
                        loc = loc(1);
                        if version("-release") == "2022b"
                            pt = img2world2d(loc.Centroid,cam.getTForm(),cam.cam_IS);
                        else
                            % matlab 2021a stuff here
                            pt = pointsToWorld(cam.cam_IS,cam.cam_R,cam.cam_T,loc.Centroid);
                        end
        
                        ballCoords = adjust_ball_pos(cam, [pt(1) pt(2) 0]);
                        ballCoords = rob2grid * [ballCoords(1); ballCoords(2); ballCoords(3); 1];
            
                        ballCoords = ballCoords(1:3);
        
                        if pos_in_bounds(ballCoords)
                            locs(i,1:3) = ballCoords.';
                            color(i) = c;
                            i = i + 1;
                        end
                    end
                end

                curr_task = "Grab Ball"

            case "Grab Ball"
                if height(locs) > 0
                    i = 1;
                    for b = locs.'
                        robot.openGripper();

                        b = [b(1) b(2) b(3) 1];

                        final_pos = [0 140 100 1];

                        color(i)

                        switch color(i)
                            case "red"
                                final_pos = [6 194 116 1];
                            case "orange"
                                final_pos = [86 178 123 1];
                            case "yellow"
                                final_pos = [4 -178 106 1];
                            case "green"
                                final_pos = [91 -168 119 1];
                        end

                        aboveBallCoords = b + [0 0 50 0];
                        aboveBallCoords = aboveBallCoords(1:3);

                        aboveFinalCoords = b + [0 0 120 0];


                        curr_loc = robot.measured_cp();
                        curr_loc = curr_loc(1:3,4).';
            
                        above_ball = tplanner.quintic_traj(curr_loc, aboveBallCoords, [0 0 0], [0 0 -10], [0 0 0], [0 0 0], 3);
                        to_ball = tplanner.quintic_traj(aboveBallCoords, b, [0 0 10], [0 0 0], [0 0 0], [0 0 0], 3);
                        to_above_final = tplanner.quintic_traj(b, aboveFinalCoords, [0 0 0], [0 0 -10], [0 0 0], [0 0 0], 3);
                        to_final = tplanner.quintic_traj(aboveFinalCoords, final_pos, [0 0 10], [0 0 0], [0 0 0], [0 0 0], 3);
            
                        robot.run_trajectory(above_ball, 3, 1);
                        robot.run_trajectory(to_ball, 3, 1);
 
                        robot.closeGripper();

                        robot.run_trajectory(to_above_final, 3, 1);
                        robot.run_trajectory(to_final, 3, 1);

                        robot.openGripper();
                        pause(2)
                        i = i + 1;
                    end
                end

                locs = [];
                color = [""];
                robot.interpolate_jp([0 0 0], 3000);
                
                pause(3);
                curr_task = "Image Processing";
        end

        
        
        
        
        %imshow(colorPart);  
    

%         
%         
    end
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()

function new_pose = adjust_ball_pos(cam, pos)
cam_pose = inv(cam.cam_pose);
cam_pose = cam_pose(1:3, 4);
cam_pose = [cam_pose(1); -cam_pose(2); cam_pose(3)];

r = 10;
h = cam_pose(3);

v1 = cam_pose - pos.';
v1_mag = norm(v1);

alpha = abs(r * (v1_mag / h));
new_pose = cam_pose - (v1 * ((v1_mag - alpha)/v1_mag));
end

function is = pos_in_bounds(pos)
    is = pos(1) > 40 && pos(1) < 160 && pos(2) > -130 && pos(2) < 130;
end

function empty_cb(~,~,~)
end

function sing_cb(msg)
    disp(msg)
end
