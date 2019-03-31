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
function [Feature] = FuncPlotFigure_DSLAM3D(GMap)

X = GMap.st;

Feature = [0,0,0];
FID = GMap.Ref(1);

n = length(X(:,1));
i = 1;
while i<=n;
    if X(i,1)==GMap.Ref(2);
        Feature = [Feature;[X(i,2),0,0]];
        FID = [FID;X(i,1)];
        i = i+1;
    elseif X(i,1)==GMap.Ref(3);
        Feature = [Feature;[X(i:i+1,2)',0]];
        FID = [FID;X(i,1)];
        i = i+2;
    else
        Feature = [Feature;X(i:i+2,2)'];
        FID = [FID;X(i,1)];
        i = i+3;
    end;
end;

%%
[FB,FIX] = sort(FID);
Feature = Feature(FIX,:);

%%
hold on;
plot3(Feature(:,1),Feature(:,2),Feature(:,3),'r.');
axis equal;

%%
Feature = [FB,Feature];


