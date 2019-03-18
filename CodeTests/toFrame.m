function [pf, PFf, PFp] = toFrame(F , p)
%   TOFRAME transform point P from global frame to frame F
%
%   In:
%       F :     reference frame         F = [fx ; fy ; falpha]
%       p :     point in global frame   p = [px ; py]
%   Out:
%       pf:     point in frame F
%       PFf:   Jacobian wrt F
%       PFp:   Jacobian wrt p
t = F(1:2);
a = F(3);
R = [cos(a) -sin(a) ;sin(a) cos(a)];
pf = R'*(p-t);
if nargout>1 
    
    %  Jacobians requested
    
    px = p(1);
    py = p(2);
    x = t(1);
    y = t(2);
    
    PFf = [...
        [-cos(a),-sin(a),   cos(a)*(py-y)-sin(a)*(px-x)]
        [  sin(a),-cos(a),-cos(a)*(px-x)-sin(a)*(py-y)]];
    
    PFp = R';
end
end

function f()
%
% Symbolic code below−−Generation and/or test of Jacobians
%−Enable 'cell mode' to use this section
%−Left−click once on the code below−the cell should turn yello
w%−Type ctrl+enter (Windows, Linux) or Cmd+enter (MacOSX) to execute
%−Check the Jacobian results in the Command Window.
syms x y a px py real
F = [x y a]';
p = [px py]';
pf = toFrame(F, p);
PFf = jacobian(pf, F)
end
    