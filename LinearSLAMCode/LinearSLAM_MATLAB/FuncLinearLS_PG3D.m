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
function [GMap] = FuncLinearLS_PG3D(LM1,LM2)

n1 = length(LM1.st(:,1));
n2 = length(LM2.st(:,1));

b = [LM1.st(:,2);LM2.st(:,2)];

I = sparse(n1+n2,n1+n2);
I(1:n1,1:n1) = LM1.I;
I(n1+1:n1+n2,n1+1:n1+n2) = LM2.I;


ID1 = (1:1:n1)';
ID2 = (1:1:n1)';
Val(1:n1,1) = 1;
XGID = LM1.st(:,1);
Xn = n1;

i = 1;
while i<=n2;
    Pn = LM2.st(i,1);
    a = find(XGID==Pn);
    
    if a;
        nn = a(end);
        
        ID1 = [ID1;n1+i;n1+i+1;n1+i+2;n1+i+3;n1+i+4;n1+i+5];
        ID2 = [ID2;a];
        Val = [Val;1;1;1;1;1;1];
        %%
        for jj = 3:5;
            Err = b(n1+i+jj,1)-b(nn-5+jj,1);
            if Err>pi;
                b(n1+i+jj,1) = b(n1+i+jj,1)-2*pi;
            elseif Err<-pi;
                b(n1+i+jj,1) = b(n1+i+jj,1)+2*pi;
            end;
%             Err = b(n1+i+jj,1)-b(nn-5+jj,1);
        end;
        
    else        
        XGID = [XGID;Pn;Pn;Pn;Pn;Pn;Pn];        
        ID1 = [ID1;n1+i;n1+i+1;n1+i+2;n1+i+3;n1+i+4;n1+i+5];
        ID2 = [ID2;Xn+1;Xn+2;Xn+3;Xn+4;Xn+5;Xn+6];
        Val = [Val;1;1;1;1;1;1];
        Xn = Xn+6;
    end;
    i = i+6;
end;
    
A = sparse(ID1,ID2,Val);

XG = (A'*I*A)\(A'*I*b);

GMap.I = A'*I*A;
GMap.st = [XGID,XG];
GMap.Ref = LM2.Ref;
GMap.sp = LM1.sp;
