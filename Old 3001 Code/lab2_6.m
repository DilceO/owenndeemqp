function lab2_6()
    % Plot Data
%     data = csvread("triangle.csv")
%     
%     hold on
%     plot(data(:,2))
%     plot(data(:,1))
%     plot(data(:,3))
% 
%     legend("Joint 0", "Joint 1", "Joint 2")
%     ylabel("Joint Rotation, Degrees")
%     xlabel("Time, Milliseconds")
%     hold off

%    hold on
%    plot(data(:,4))
%    plot(data(:,6))
% 
%    legend("X Axis", "Z Axis")
%    ylabel("End Effector Position, mm")
%    xlabel("Time, ms")
%    hold off

% hold on
% plot(data(1:height(data) - 10,4), data(1:height(data) - 10,5))
% scatter(67.5, -1.6, 50,"blue", "filled")
% scatter(97, -3, 50, "green", "filled")
% scatter(160, -4, 50, "red",'filled')
% xlim([60, 170])
% ylim([0, 70])
% axis equal
% xlabel("X Axis")
% ylabel("Y Axis")
% hold off

data = csvread("Transforms.csv");
data = data(:, end-3:end)

hold on
scatter3(data(:,1), data(:,2), data(:,3), 36, "red", "filled");
grid on
m = rms(data)
scatter3(m(1), m(2), m(3), 36, "green", "filled");
legend("End Effector Position", "Average")
end