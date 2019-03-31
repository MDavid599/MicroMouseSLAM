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
function [LM2,n] = LMJ_DSLAM3D_Precondition(LM,LocalMapNum)

LM2 = {};
i = 1;
n = 0;
Ref = [];
while i<=LocalMapNum;
    if i==LocalMapNum;
        n = n+1;
        [GMap] = FuncDMap_DSLAM3D(LM{i},Ref);
        LM2{n} = GMap;
        i = i+1;
    else
        c = [];
        j = 0;
        while length(c)<3 && i+j<=LocalMapNum;        
            if j==0;
                GMap = LM{i};
            else
                [GMap_End] = FuncTransform_PF3D(GMap,LM{i+j}.Ref);
                [GMap] = FuncLinearLS_PF3D(GMap_End,LM{i+j});            
            end;
            j = j+1;
            c = FuncDSLAM_Ref_3D(GMap,LM{i+j});
        end;
        if n==0;
            Ref = c(1:3);
            [GMap] = FuncDMap_DSLAM3D(GMap,Ref);
        else            
            [GMap] = FuncDMap_DSLAM3D(GMap,Ref);
            Ref = c(1:3);
        end;
            
        n = n+1;
        LM2{n} = GMap;
        i = i+j;
    end;
end;
    
  
