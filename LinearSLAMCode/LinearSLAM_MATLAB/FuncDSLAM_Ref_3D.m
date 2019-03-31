function Ref = FuncDSLAM_Ref_3D(LM1,LM2)

cc = intersect(LM1.st(:,1),LM2.st(:,1));
n = length(cc);
Ref = [];
Threshold = 0.2; %rad

if n<3;
    Ref = cc;
else
    for i=1:n;
        for j=1:n;
            for k=1:n;
                if i~=j~=k;
                    a = find(LM2.st(:,1)==cc(i));
                    b = find(LM2.st(:,1)==cc(j));
                    c = find(LM2.st(:,1)==cc(k));
                    t = LM2.st(a(:),2);
                    t2 = LM2.st(b(:),2);
                    t3 = LM2.st(c(:),2);    
                    Omega = acos((t2-t)'*(t3-t)/(norm(t2-t)*norm(t3-t)));
                    if Omega>=Threshold;
                        Ref = cc([i,j,k]);
                    end;
                end;
            end;
        end;
    end;
end;