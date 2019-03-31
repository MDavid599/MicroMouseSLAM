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
function [GMap_End] = FuncTransform_DSLAM3D(GMap,Ref)

if GMap.Ref == Ref;
    
    GMap_End = GMap;
    
else
    
    n = length(GMap.st(:,1))+6;
    st = zeros(n,2);
    c1 = find(GMap.st(:,1)==GMap.Ref(2));
    c2 = find(GMap.st(:,1)==GMap.Ref(3));
    if c1<c2(1);
        cc = [1:c1,c1+3:c2(2)+2,c2(2)+4:n-3];
        st(cc,:) = GMap.st(:,:);
        st(c1+1:c1+2,1) = GMap.Ref(2);
        st(c2(2)+3,1) = GMap.Ref(3);
        st(n-2:n,1) = GMap.Ref(1);
    elseif c1>c2(1);
        cc = [1:c2(2),c2(2)+2:c1+1,c1+4:n-3];
        st(cc,:) = GMap.st(:,:);
        st(c2(2)+1,1) = GMap.Ref(3);
        st(c1+2:c1+3,1) = GMap.Ref(2);
        st(n-2:n,1) = GMap.Ref(1);
    end;   
    GMap.st = st;
    
    a = find(GMap.st(:,1)==Ref(1));
    b = find(GMap.st(:,1)==Ref(2));
    c = find(GMap.st(:,1)==Ref(3));
    t = GMap.st(a(:),2);
    t2 = GMap.st(b(:),2);
    t3 = GMap.st(c(:),2);
    v1 = t2-t;
    v3 = cross(v1,t3-t);
    v2 = cross(v3,v1);
    R(1,:) = v1/norm(v1);
    R(2,:) = v2/norm(v2);
    R(3,:) = v3/norm(v3);
    
%     a = find(GMap.st(:,1)==Ref(1));
%     b = find(GMap.st(:,1)==Ref(2));
%     t = GMap.st(a(:),2);
%     t2 = GMap.st(b(:),2);
%     dt = t2-t;
%     phi = atan2(dt(2),dt(1));
%     R = FuncRMatrix2D(phi);

    n = length(GMap.st(:,1));
    
    GMap_End.st = zeros(n,2);
    GMap_End.st(:,1) = GMap.st(:,1);
    GMap_End.Ref = Ref;
    
    KID = [];
    i = 1;
    while i<=n;
        GMap_End.st(i:i+2,2) = R*(GMap.st(i:i+2,2)-t);
        if i==a(1);
            GMap_End.st(i:i+2,2) = 0;
        elseif i==b(1);
        	GMap_End.st(i+1:i+2,2) = 0;
            KID = [KID;i];
        elseif i==c(1);
            GMap_End.st(i+2,2) = 0;
            KID = [KID;i;i+1];
        else
            KID = [KID;i;i+1;i+2];
        end;
        i = i+3;
            
%         GMap_End.st(i:i+1,2) = R*(GMap.st(i:i+1,2)-t);        
%         if i==a(1);
%             GMap_End.st(i:i+1,2) = 0;
%         elseif i==b(1);
%             GMap_End.st(i+1,2) = 0;
%             KID = [KID;i];
%         else
%             KID = [KID;i;i+1];
%         end;
%         i = i+2;
    end;

%%
%     a = find(GMap_End.st(:,1)==GMap.Ref(1));
%     b = find(GMap_End.st(:,1)==GMap.Ref(2));
%     
%     t = GMap_End.st(a(:),2);
%     t2 = GMap_End.st(b(:),2);
%     dt = t2-t;
%     phi = atan2(dt(2),dt(1));
%     R = FuncRMatrix2D(phi);
%     
%     dR = FuncdR2D(phi);
%     Ddt = dt'*dt;
%     dX = dt(2)/Ddt;
%     dX2 = -dt(2)/Ddt;
%     dY = -dt(1)/Ddt;
%     dY2 = dt(1)/Ddt;
%     dRdX = dR*dX;
%     dRdX2 = dR*dX2;
%     dRdY = dR*dY;
%     dRdY2 = dR*dY2;
    
    a = find(GMap_End.st(:,1)==GMap.Ref(1));
    b = find(GMap_End.st(:,1)==GMap.Ref(2));
    c = find(GMap_End.st(:,1)==GMap.Ref(3));
    t = GMap_End.st(a(:),2);
    t2 = GMap_End.st(b(:),2);
    t3 = GMap_End.st(c(:),2);
    v1 = t2-t;
    v3 = cross(v1,t3-t);
    v2 = cross(v3,v1);
    R(1,:) = v1/norm(v1);
    R(2,:) = v2/norm(v2);
    R(3,:) = v3/norm(v3);
    
    nv1 = norm(v1);
    dnv1dn2v1 = 1/(2*nv1);
    dn2v1dv1 = 2*v1';
    dnv1dv1 = dnv1dn2v1*dn2v1dv1;
    dR1Tdv1 = (eye(3)*nv1-v1*dnv1dv1)/(nv1^2);
    dv1dt = -eye(3);
    dv1dt2 = eye(3);
    dR1Tdt = dR1Tdv1*dv1dt;
    dR1Tdt2 = dR1Tdv1*dv1dt2;
    
    nv3 = norm(v3);
    dnv3dn3v3 = 1/(2*nv3);
    dn3v3dv3 = 2*v3';
    dnv3dv3 = dnv3dn3v3*dn3v3dv3;
    dR3Tdv3 = (eye(3)*nv3-v3*dnv3dv3)/(nv3^2);    
    Cv1 = CROSS_3D(v1);
    CFy = CROSS_3D(t3-t);
    dv3dFy = Cv1;
    dv3dv1 = -CFy;
    dFydt = -eye(3);
    dFydt3 = eye(3);
    dv3dt2 = dv3dv1*dv1dt2;
    dv3dt3 = dv3dFy*dFydt3;
    dv3dt = dv3dv1*dv1dt+dv3dFy*dFydt;
    dR3Tdt = dR3Tdv3*dv3dt;
    dR3Tdt2 = dR3Tdv3*dv3dt2;
    dR3Tdt3 = dR3Tdv3*dv3dt3;
    
    nv2 = norm(v2);
    dnv2dn2v2 = 1/(2*nv2);
    dn2v2dv2 = 2*v2';
    dnv2dv2 = dnv2dn2v2*dn2v2dv2;
    dR2Tdv2 = (eye(3)*nv2-v2*dnv2dv2)/(nv2^2);
    %v2 = cross(v3,v1);
    Cv3 = CROSS_3D(v3);
    dv2dv1 = Cv3;
    dv2dv3 = -Cv1;
    dv2dt = dv2dv1*dv1dt+dv2dv3*dv3dt;
    dv2dt2 = dv2dv1*dv1dt2+dv2dv3*dv3dt2;
    dv2dt3 = dv2dv3*dv3dt3;
    dR2Tdt = dR2Tdv2*dv2dt;
    dR2Tdt2 = dR2Tdv2*dv2dt2;
    dR2Tdt3 = dR2Tdv2*dv2dt3;

    ID1 = [];
    ID2 = [];
    Val = [];
    
    i = 1;
    while i<=n;
%         ID1 = [ID1;i;i+1;i;i+1;i;i+1;i;i+1;i;i+1;i;i+1;i;i+1;i;i+1];
%         ID2 = [ID2;a(1);a(1);a(2);a(2);a(1);a(1);a(2);a(2);b(1);b(1);b(2);b(2);i;i;i+1;i+1];
%         Val = [Val;-R(:,1);-R(:,2);dRdX*(GMap_End.st(i:i+1,2)-t);dRdY*(GMap_End.st(i:i+1,2)-t);dRdX2*(GMap_End.st(i:i+1,2)-t);dRdY2*(GMap_End.st(i:i+1,2)-t);R(:,1);R(:,2)];
%         i = i+2;
        tt = GMap_End.st(i:i+2,2)-t;
        ID1 = [ID1;i;i+1;i+2; i;i+1;i+2; i;i+1;i+2;  i;i+1;i+2; i;i+1;i+2; i;i+1;i+2;  i;i;i; i;i;i;  i+1;i+1;i+1; i+1;i+1;i+1; i+1;i+1;i+1;  i+2;i+2;i+2; i+2;i+2;i+2; i+2;i+2;i+2];
        ID2 = [ID2;a(1);a(1);a(1); a(2);a(2);a(2); a(3);a(3);a(3);  i;i;i; i+1;i+1;i+1; i+2;i+2;i+2;  a; b;  a; b; c;  a; b; c];
        Val = [Val;-R(:,1); -R(:,2); -R(:,3);  R(:,1); R(:,2); R(:,3);  dR1Tdt'*tt; dR1Tdt2'*tt;  dR2Tdt'*tt; dR2Tdt2'*tt; dR2Tdt3'*tt;  dR3Tdt'*tt; dR3Tdt2'*tt; dR3Tdt3'*tt];
        i = i+3;
    end;

    J = sparse(ID1,ID2,Val);
    J = J(cc,KID);
    GMap_End.I = J'*GMap.I*J;
    GMap_End.st = GMap_End.st(KID,:);
    
end;

