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
function [GMap_End] = FuncTransform_PF3D(GMap,Ref)

%%
if GMap.Ref == Ref;
    GMap_End = GMap;
else
%%    
    a = find(GMap.st(:,1)==-Ref);

    t = GMap.st(a(1:3),2);
    Alpha = GMap.st(a(4),2);
    Beta = GMap.st(a(5),2);
    Gamma = GMap.st(a(6),2);
    R = RMatrixYPR22(Alpha,Beta,Gamma);

    n = length(GMap.st(:,1));
    
    GMap_End.st = zeros(n,2);
    GMap_End.st(:,1) = GMap.st(:,1);
    GMap_End.st(a,1) = -GMap.Ref;
    GMap_End.Ref = Ref;
    
    i = 1;
    while i<=n;
        if GMap.st(i,1) <= 0;
            if i == a(1);
                GMap_End.st(i:i+2,2) = -R*t;
                [GMap_End.st(i+3,2),GMap_End.st(i+4,2),GMap_End.st(i+5,2)] = InvRotMatrixYPR22(R');
            else
                GMap_End.st(i:i+2,2) = R*(GMap.st(i:i+2,2)-t);
                R2 = RMatrixYPR22(GMap.st(i+3,2),GMap.st(i+4,2),GMap.st(i+5,2));
                [GMap_End.st(i+3,2),GMap_End.st(i+4,2),GMap_End.st(i+5,2)] = InvRotMatrixYPR22(R2*R');
            end;
            i = i+6;
        else
            GMap_End.st(i:i+2,2) = R*(GMap.st(i:i+2,2)-t);
            i = i+3;
        end;
    end;

%%
    t = GMap_End.st(a(1:3),2);
    Alpha = GMap_End.st(a(4),2);
    Beta = GMap_End.st(a(5),2);
    Gamma = GMap_End.st(a(6),2);
    %%
    RZ = FuncRZ(Alpha);
    RY = FuncRY(Beta);
    RX = FuncRX(Gamma);
    R = FuncR(RX,RY,RZ);
    dRZdA = FuncdRZdA(Alpha);
    dRYdB = FuncdRYdB(Beta);
    dRXdG = FuncdRXdG(Gamma);
    dRdA = FuncR(RX,RY,dRZdA);
    dRdB = FuncR(RX,dRYdB,RZ);
    dRdG = FuncR(dRXdG,RY,RZ);
    %%
    dA = FuncdRi(dRdA',R');
    dB = FuncdRi(dRdB',R');
    dG = FuncdRi(dRdG',R');
    %%
    ID1 = [];
    ID2 = [];
    Val = [];

    i = 1;
    while i<=n;
        if GMap_End.st(i,1) <= 0;
            if i == a(1);
                ID1 = [ID1;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i+3;i+4;i+5;i+3;i+4;i+5;i+3;i+4;i+5];
                ID2 = [ID2;a(1);a(1);a(1);a(2);a(2);a(2);a(3);a(3);a(3);a(4);a(4);a(4);a(5);a(5);a(5);a(6);a(6);a(6);a(4);a(4);a(4);a(5);a(5);a(5);a(6);a(6);a(6)];
                Val = [Val;-R(:,1);-R(:,2);-R(:,3);-dRdA*t;-dRdB*t;-dRdG*t;dA;dB;dG];
            else                
                %%
                t2 = GMap_End.st(i:i+2,2);        
                Alpha2 = GMap_End.st(i+3,2);   
                Beta2 = GMap_End.st(i+4,2);
                Gamma2 = GMap_End.st(i+5,2);
                %%            
                RZ2 = FuncRZ(Alpha2);
                RY2 = FuncRY(Beta2);
                RX2 = FuncRX(Gamma2);
                R2 = FuncR(RX2,RY2,RZ2);
            
                Ri = R2*R';
            
                dRZ2dA2 = FuncdRZdA(Alpha2);
                dRY2dB2 = FuncdRYdB(Beta2);
                dRX2dG2 = FuncdRXdG(Gamma2);
                dR2dA2 = FuncR(RX2,RY2,dRZ2dA2);
                dR2dB2 = FuncR(RX2,dRY2dB2,RZ2);
                dR2dG2 = FuncR(dRX2dG2,RY2,RZ2);
                %%        
                dRidA2 = dR2dA2*R';
                dRidB2 = dR2dB2*R';
                dRidG2 = dR2dG2*R';
                dRidA = R2*dRdA';
                dRidB = R2*dRdB';
                dRidG = R2*dRdG';
                %%
                ddA2 = FuncdRi(dRidA2,Ri);
                ddB2 = FuncdRi(dRidB2,Ri);
                ddG2 = FuncdRi(dRidG2,Ri);
                ddA = FuncdRi(dRidA,Ri);
                ddB = FuncdRi(dRidB,Ri);
                ddG = FuncdRi(dRidG,Ri);
                %%        
                ID1 = [ID1;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i+3;i+4;i+5;i+3;i+4;i+5;i+3;i+4;i+5;i+3;i+4;i+5;i+3;i+4;i+5;i+3;i+4;i+5];
                ID2 = [ID2;a(1);a(1);a(1);a(2);a(2);a(2);a(3);a(3);a(3);a(4);a(4);a(4);a(5);a(5);a(5);a(6);a(6);a(6);i;i;i;i+1;i+1;i+1;i+2;i+2;i+2;a(4);a(4);a(4);a(5);a(5);a(5);a(6);a(6);a(6);i+3;i+3;i+3;i+4;i+4;i+4;i+5;i+5;i+5];
                Val = [Val;-R(:,1);-R(:,2);-R(:,3);dRdA*(t2-t);dRdB*(t2-t);dRdG*(t2-t);R(:,1);R(:,2);R(:,3);ddA;ddB;ddG;ddA2;ddB2;ddG2]; 
            end;
            i = i+6;
        else
            F = GMap_End.st(i:i+2,2); 
            ID1 = [ID1;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2;i;i+1;i+2];
            ID2 = [ID2;a(1);a(1);a(1);a(2);a(2);a(2);a(3);a(3);a(3);a(4);a(4);a(4);a(5);a(5);a(5);a(6);a(6);a(6);i;i;i;i+1;i+1;i+1;i+2;i+2;i+2];
            Val = [Val;-R(:,1);-R(:,2);-R(:,3);dRdA*(F-t);dRdB*(F-t);dRdG*(F-t);R(:,1);R(:,2);R(:,3)];
            i = i+3;
        end;
        
    end;

    J = sparse(ID1,ID2,Val);
    GMap_End.I = J'*GMap.I*J;

end;
    
