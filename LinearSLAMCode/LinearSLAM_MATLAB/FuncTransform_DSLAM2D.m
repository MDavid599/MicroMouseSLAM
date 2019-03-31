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
function [GMap_End] = FuncTransform_DSLAM2D(GMap,Ref)

if GMap.Ref == Ref;
    
    GMap_End = GMap;
    
else
    
    n = length(GMap.st(:,1))+3;
    st = zeros(n,2);
    c = find(GMap.st(:,1)==GMap.Ref(2));
    cc = [1:c,c+2:n-2];
    st(cc,:) = GMap.st(:,:);
    st(c+1,1) = GMap.Ref(2);
    st(n-1:n,1) = GMap.Ref(1);    
    GMap.st = st;
    
    a = find(GMap.st(:,1)==Ref(1));
    b = find(GMap.st(:,1)==Ref(2));
    t = GMap.st(a(:),2);
    t2 = GMap.st(b(:),2);
    dt = t2-t;
    phi = atan2(dt(2),dt(1));
    R = FuncRMatrix2D(phi);

    n = length(GMap.st(:,1));
    
    GMap_End.st = zeros(n,2);
    GMap_End.st(:,1) = GMap.st(:,1);
    GMap_End.Ref = Ref;
    
    KID = [];
    i = 1;
    while i<=n;
        GMap_End.st(i:i+1,2) = R*(GMap.st(i:i+1,2)-t);        
        if i==a(1);
            GMap_End.st(i:i+1,2) = 0;
        elseif i==b(1);
            GMap_End.st(i+1,2) = 0;
            KID = [KID;i];
        else
            KID = [KID;i;i+1];
        end;
        i = i+2;
    end;

%%
    a = find(GMap_End.st(:,1)==GMap.Ref(1));
    b = find(GMap_End.st(:,1)==GMap.Ref(2));
    
    t = GMap_End.st(a(:),2);
    t2 = GMap_End.st(b(:),2);
    dt = t2-t;
    phi = atan2(dt(2),dt(1));
    R = FuncRMatrix2D(phi);
    
    dR = FuncdR2D(phi);
    Ddt = dt'*dt;
    dX = dt(2)/Ddt;
    dX2 = -dt(2)/Ddt;
    dY = -dt(1)/Ddt;
    dY2 = dt(1)/Ddt;
    dRdX = dR*dX;
    dRdX2 = dR*dX2;
    dRdY = dR*dY;
    dRdY2 = dR*dY2;

    ID1 = [];
    ID2 = [];
    Val = [];
    
    i = 1;
    while i<=n;
        ID1 = [ID1;i;i+1;i;i+1;i;i+1;i;i+1;i;i+1;i;i+1;i;i+1;i;i+1];
        ID2 = [ID2;a(1);a(1);a(2);a(2);a(1);a(1);a(2);a(2);b(1);b(1);b(2);b(2);i;i;i+1;i+1];
        Val = [Val;-R(:,1);-R(:,2);dRdX*(GMap_End.st(i:i+1,2)-t);dRdY*(GMap_End.st(i:i+1,2)-t);dRdX2*(GMap_End.st(i:i+1,2)-t);dRdY2*(GMap_End.st(i:i+1,2)-t);R(:,1);R(:,2)];
        i = i+2;
    end;

    J = sparse(ID1,ID2,Val);
    J = J(cc,KID);
    GMap_End.I = J'*GMap.I*J;
    GMap_End.st = GMap_End.st(KID,:);
    
end;
