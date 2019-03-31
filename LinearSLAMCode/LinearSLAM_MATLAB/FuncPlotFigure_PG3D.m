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
function [Pose] = FuncPlotFigure_PG3D(GMap)

X = GMap.st;

Pose = [];
PID = [];

n = length(X(:,1));
i = 1;
while i<=n;
    PID = [PID;X(i,1)];
	Pose = [Pose;X(i:i+5,2)'];
	i = i+6;
end;

%%
[B,IX] = sort(PID);
Pose = Pose(IX,:);

%%
hold on;
plot3(Pose(:,1),Pose(:,2),Pose(:,3),'r.-');
axis equal;

Pose = [B,Pose];

