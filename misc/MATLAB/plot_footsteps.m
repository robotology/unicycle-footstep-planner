function plot_footsteps(xL, yL, angleLeft, xR, yR, angleRight, alpha)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
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
%% SET FOOT SIZE
x = [-0.015, -0.015, 0.045, 0.045];
y = [-0.015,  0.015, 0.015, -0.015];

posLeft = [xL, yL]';
posRight = [xR, yR]';

%% PLOT FOOTPATHS
for i = 1:length(posLeft)
    % plot left footpath
    left_foot_transform = hgtransform;
    left_foot_transform.Matrix = makehgtform('translate', [posLeft(:,i);0],...
                                             'zrotate', angleLeft(i));
    patch('XData',x,'YData',y,'FaceColor','green','FaceAlpha',alpha,'Parent',left_foot_transform);
end

for i = 1:length(posRight)
    % plot right footpath
    right_foot_transform = hgtransform;
    right_foot_transform.Matrix = makehgtform('translate', [posRight(:,i);0],...
                                              'zrotate', angleRight(i));
    patch('XData',x,'YData',y,'FaceColor','yellow','FaceAlpha',alpha,'Parent',right_foot_transform);
end
end
