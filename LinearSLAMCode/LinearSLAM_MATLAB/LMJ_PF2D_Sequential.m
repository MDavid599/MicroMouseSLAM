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
function [GMap] = LMJ_PF2D_Sequential(LM,LocalMapNum)

for i=1:LocalMapNum;
    
    fprintf('Join Local Map %d\n', i);
    
	if i==1;

        GMap = LM{i};
	else
        [GMap_End] = FuncTransform_PF2D(GMap,LM{i}.Ref);
        [GMap] = FuncLinearLS_PF2D(GMap_End,LM{i});
	end;
end;

%%
a = find(GMap.st(:,1)<=0);
m = min(-GMap.st(a(:),1));
if GMap.Ref>m;
    [GMap] = FuncTransform_PF2D(GMap,m);
end;    
