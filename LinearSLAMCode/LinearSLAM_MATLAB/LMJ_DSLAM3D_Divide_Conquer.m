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
function [GMap,LM_L] = LMJ_DSLAM3D_Divide_Conquer(LM,LocalMapNum)

L = 1;
LM_L{L} = LM;
clear LM;

while LocalMapNum>1;
    N2 = rem(LocalMapNum,2);
    LocalMapNum = ceil(LocalMapNum/2);
    
    for i=1:LocalMapNum;
        
        if i<LocalMapNum;
            NumLM = 2;
        elseif i==LocalMapNum;
            if N2;
                NumLM = 1;
            else
                NumLM = 2;
            end;
        end;
        PreRef = [];
        for j=1:NumLM;                        
            fprintf('Join Level %d Local Map %d\n', L, 2*(i-1)+j);    
            if j==1;
                GMap = LM_L{L}{2*(i-1)+j};
                if L==1;
                    PreRef = GMap.Ref;
                else
                    PreRef = GMap.PreRef;
                end;
            else
                [GMap_End] = FuncTransform_DSLAM3D(GMap,LM_L{L}{2*(i-1)+j}.Ref);
                [GMap] = FuncLinearLS_DSLAM3D(GMap_End,LM_L{L}{2*(i-1)+j});
            end;
        end;
        
        if rem(i,2) == 0;
            [GMap] = FuncTransform_DSLAM3D(GMap,PreRef);
        end;
        GMap.PreRef = PreRef;
        
        LM_L{L+1}{i} = GMap;        
        fprintf('Generate Level %d Local Map %d\n\n', L+1, i);
        
    end;
    
    L = L+1;
    
end;

