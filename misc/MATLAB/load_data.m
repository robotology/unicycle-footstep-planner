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

clear all; clc;

% Add first run
left1 = importdata('footstepsL1.txt');
right1 = importdata('footstepsR1.txt');
dcmPos1 = importdata('dcmPos1.txt');
mergePoints1 = importdata('mergePoints1.txt');
alpha1 = 0.3;

% Add second run
left2 = importdata('footstepsL2.txt');
right2 = importdata('footstepsR2.txt');
dcmPos2 = importdata('dcmPos2.txt');
mergePoints2 = importdata('mergePoints2.txt');
alpha2 = 0.6;

% Add third run
left3 = importdata('footstepsL3.txt');
right3 = importdata('footstepsR3.txt');
dcmPos3 = importdata('dcmPos3.txt');
mergePoints3 = importdata('mergePoints3.txt');
alpha3 = 1;

trajectoryStartTime = [0, mergePoints1(2),...
    mergePoints1(2) + mergePoints2(2)] + 1;