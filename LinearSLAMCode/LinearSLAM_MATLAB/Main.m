% ===============================================================================================
% Linear SLAM: A Linear Solution to the Pose Feature and Pose Graph SLAM based on Submap Joining 
% Version: 1.0
% ===============================================================================================
% 
% Copyright (C) 2013 Liang Zhao, Shoudong Huang and Gamini Dissanayake
% University of Technology, Sydney, Australia
% 
% Authors:  Liang Zhao         -- Liang.Zhao-1@uts.edu.au 
%           Shoudong Huang     -- Shoudong.Huang@uts.edu.au
%           Gamini Dissanayake -- Gamini.Dissanayake@uts.edu.au
% 
%           Centre for Autonomous Systems
%           Faculty of Engineering and Information Technology
%           University of Technology, Sydney
%           NSW 2007, Australia
% 
% License
% 
% Linear SLAM by Liang Zhao, Shoudong Huang, Gamini Dissanayake is licensed under a 
% Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
% 
% Please contact Liang Zhao {Liang.Zhao-1@uts.edu.au} if you have any questions/comments about the code.
 
%%
clc;
clear;

%% Map Joining Method

% Method = 'Sequential';
Method = 'Divide&Conquer';

%% Dataset Type

% DataType = '2D Pose Feature';
% DataType = '3D Pose Feature';
% DataType = '2D Pose Graph';
DataType = '3D Pose Graph';
% DataType = '2D DSLAM';
% DataType = '3D DSLAM';

%% Datasets

    %% 2D Pose Feature & DSLAM Datasets

% Dataset = 'VicPark_200_local_maps';
% Dataset = 'VicPark_6898_local_maps';
% Dataset = 'DLR_200_local_maps';
% Dataset = 'DLR_3298_local_maps';
% Dataset = '8240_data_50_local_maps';
% Dataset = '35188_data_700_local_maps';

    %% 3D Pose Feature & DSLAM Datasets

% Dataset = 'Simu_3D_870_Loop';

    %% 2D Pose Graph Datasets

% Dataset = 'Intel';
% Dataset = 'manhattanOlson3500';
% Dataset = 'city10000';

    %% 3D Pose Graph Datasets

Dataset = 'parking-garage';
% Dataset = 'sphere2500';

    %% Test Your Own Dataset
% Dataset = 'test_your_own_dataset';

%% Direction and Local Map Number
switch Dataset;
    case 'VicPark_200_local_maps';
        LocalMapNum = 200;
        Direction = 'VicPark_200_local_maps/';
    case 'VicPark_6898_local_maps';
        LocalMapNum = 6898;
        Direction = 'VicPark_6898_local_maps/';
    case 'DLR_200_local_maps';
        LocalMapNum = 200;
        Direction = 'DLR_200_local_maps/';
    case 'DLR_3298_local_maps';
        LocalMapNum = 3298;
        Direction = 'DLR_3298_local_maps/';
    case '8240_data_50_local_maps';
        LocalMapNum = 50;
        Direction = '8240_data_50_local_maps/';
    case '35188_data_700_local_maps';
        LocalMapNum = 700;
        Direction = '35188_data_700_local_maps/';
    case 'Simu_3D_870_Loop';
        LocalMapNum = 870;
%         LocalMapNum = 8;
        Direction = 'Simu_3D_870_Loop/';
    case 'Intel';
        LocalMapNum = 943-1;
        Direction = 'Intel/';
    case 'manhattanOlson3500';
        LocalMapNum = 3500-1;
        Direction = 'manhattanOlson3500/';
    case 'city10000';
        LocalMapNum = 10000-1;
        Direction = 'city10000/';
    case 'parking-garage';
        LocalMapNum = 1661-1;
        Direction = 'parking-garage/';
    case 'sphere2500';
        LocalMapNum = 2500-1;
        Direction = 'sphere2500/';
    case 'test_your_own_dataset';
        LocalMapNum = 22; % Input your local map number
        Direction = 'test_your_own_dataset/';
end;

%% Load Files
for i=1:LocalMapNum;
    file=strcat(Direction,'localmap_',int2str(i)); 
	LM{i} = load(file);
end;

if strcmp(DataType,'2D DSLAM');    
    [LM,LocalMapNum] = LMJ_DSLAM2D_Precondition(LM,LocalMapNum);
end;

if strcmp(DataType,'3D DSLAM');    
    [LM,LocalMapNum] = LMJ_DSLAM3D_Precondition(LM,LocalMapNum);
 end;

%% Linear Map Joining
tic;

switch Method;
    case 'Sequential';
        switch DataType;
            case '2D Pose Feature';
                [GMap] = LMJ_PF2D_Sequential(LM,LocalMapNum); % 2D Pose Feature Sequential
            case '3D Pose Feature';
                [GMap] = LMJ_PF3D_Sequential(LM,LocalMapNum); % 3D Pose Feature Sequential
            case '2D Pose Graph';
                [GMap] = LMJ_PG2D_Sequential(LM,LocalMapNum); % 2D Pose Graph Sequential
            case '3D Pose Graph';
                [GMap] = LMJ_PG3D_Sequential(LM,LocalMapNum); % 3D Pose Graph Sequential
            case '2D DSLAM';                
                [GMap] = LMJ_DSLAM2D_Sequential(LM,LocalMapNum); % 2D DSLAM Sequential
            case '3D DSLAM';                
                [GMap] = LMJ_DSLAM3D_Sequential(LM,LocalMapNum); % 3D DSLAM Sequential
        end;
    case 'Divide&Conquer';
        switch DataType;
            case '2D Pose Feature';
                [GMap,Level_Map] = LMJ_PF2D_Divide_Conquer(LM,LocalMapNum); % 2D Pose Feature Divide and Conquer
            case '3D Pose Feature';
                [GMap,Level_Map] = LMJ_PF3D_Divide_Conquer(LM,LocalMapNum); % 3D Pose Feature Divide and Conquer
            case '2D Pose Graph';
                [GMap,Level_Map] = LMJ_PG2D_Divide_Conquer(LM,LocalMapNum); % 2D Pose Graph Divide and Conquer
            case '3D Pose Graph';
                [GMap,Level_Map] = LMJ_PG3D_Divide_Conquer(LM,LocalMapNum); % 3D Pose Graph Divide and Conquer
            case '2D DSLAM';
                [GMap,Level_Map] = LMJ_DSLAM2D_Divide_Conquer(LM,LocalMapNum); % 2D DSLAM Divide and Conquer
            case '3D DSLAM';
                [GMap,Level_Map] = LMJ_DSLAM3D_Divide_Conquer(LM,LocalMapNum); % 3D DSLAM Divide and Conquer
        end;
end

toc

%% Plot Result Figure
switch DataType;
    case '2D Pose Feature';
        [Pose,Feature] = FuncPlotFigure_PF2D(GMap);
    case '3D Pose Feature';
        [Pose,Feature] = FuncPlotFigure_PF3D(GMap);
    case '2D Pose Graph';
        [Pose] = FuncPlotFigure_PG2D(GMap);
    case '3D Pose Graph';
        [Pose] = FuncPlotFigure_PG3D(GMap);
    case '2D DSLAM';
        [Feature] = FuncPlotFigure_DSLAM2D(GMap);
    case '3D DSLAM';
        [Feature] = FuncPlotFigure_DSLAM3D(GMap);
end;
