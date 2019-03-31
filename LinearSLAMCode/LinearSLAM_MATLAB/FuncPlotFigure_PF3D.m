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
% 
%%
function [Pose,Feature] = FuncPlotFigure_PF3D(GMap)

X = GMap.st;

Pose = [];
Feature = [];
FID = [];
PID = [];


n = length(X(:,1));
i = 1;
while i<=n;
    if X(i,1)<=0;
        Pose = [Pose;X(i:i+5,2)'];
        PID = [PID;-X(i,1)];
        i = i+6;
    else
        Feature = [Feature;X(i:i+2,2)'];
        FID = [FID;X(i,1)];
        i = i+3;
    end;
end;

%%
[PB,PIX] = sort(PID);
Pose = Pose(PIX,:);

[FB,FIX] = sort(FID);
Feature = Feature(FIX,:);

%%
hold on;

plot3(Pose(:,1),Pose(:,2),Pose(:,3),'r.');
plot3(Feature(:,1),Feature(:,2),Feature(:,3),'k.');
axis equal;

%%
Pose = [PB,Pose];
Feature = [FB,Feature];

