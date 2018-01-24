% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
%  * @author: Giulio Romualdi
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  **/
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PLOT 2D-DCM TRAJECTORY
figure;
hold on;
% plot the first run
plot_footsteps(left1.data(:,1), left1.data(:,2), left1.data(:,3),...
    right1.data(:,1), right1.data(:,2), right1.data(:,3),...
    alpha1)
plot(dcmPos1.data(trajectoryStartTime(1):end,1), ...
    dcmPos1.data(trajectoryStartTime(1):end,2))

% plot the second run
plot_footsteps(left2.data(:,1), left2.data(:,2), left2.data(:,3),...
    right2.data(:,1), right2.data(:,2), right2.data(:,3),...
    alpha2)
plot(dcmPos2.data(trajectoryStartTime(2):end,1), ...
    dcmPos2.data(trajectoryStartTime(2):end,2))

% plot the third run
plot_footsteps(left3.data(:,1), left3.data(:,2), left3.data(:,3),...
    right3.data(:,1), right3.data(:,2), right3.data(:,3),...
    alpha3)
 plot(dcmPos3.data(trajectoryStartTime(3):end,1), ...
   dcmPos3.data(trajectoryStartTime(3):end,2))

grid on;
title('DCM planar trajectory')
xlabel('x (m)');
ylabel('y (m)');
pbaspect([1 1 1])

%%  PLOT X-COORDINATE OF THE DCM TRAJECTORY
figure;
subplot(1,2,1)
hold on;
% first run
plot([0:length(dcmPos1.data(:,2))-1]*0.01, dcmPos1.data(:,1))
% second run
plot([trajectoryStartTime(2) : length(dcmPos2.data(:,1))] * 0.01, ...
    dcmPos2.data(trajectoryStartTime(2):end,1))
% third run
plot([trajectoryStartTime(3) : length(dcmPos3.data(:,1))] * 0.01, ...
    dcmPos3.data(trajectoryStartTime(3):end,1))

grid on;
title('DCM trajectory')
xlabel('time (s)');
ylabel('x (m)');
legend('Run 1' ,'Run 2', 'Run 3');

%%  PLOT Y-COORDINATE OF THE DCM TRAJECTORY
subplot(1,2,2)
hold on;
% first run
plot([0:length(dcmPos1.data(:,2))-1]*0.01, dcmPos1.data(:,2))
% second run
plot([trajectoryStartTime(2) : length(dcmPos2.data(:,1))] * 0.01, ...
    dcmPos2.data(trajectoryStartTime(2):end,2))
% third run
plot([trajectoryStartTime(3) : length(dcmPos3.data(:,1))] * 0.01, ...
    dcmPos3.data(trajectoryStartTime(3):end,2))

grid on;
title('DCM trajectory')
xlabel('time (s)');
ylabel('y (m)');
legend('Run 1' ,'Run 2', 'Run 3');