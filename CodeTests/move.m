function [ro, ROr, ROn] = move(r, u, n)
%   MOVE Robot motion, with separated control and perturbation inputs.
%
%   In:
%       r: robot pose       r = [x ; y ; alpha]
%       u: control signal   u = [dx ; dalpha]
%       n: perturbation, additive to control signal
%   Out:
%       ro: updated robot pose
%       ROr: Jacobian   d(ro) / d(r)
%       ROn: Jacobian   d(ro) / d(n)
a = r(3);
dx = u(1) + n(1);
da = u(2) + n(2);

ao = a + da;

if ao>pi
    ao = ao-2*pi;
end

if ao<-pi
    ao = ao + 2*pi;
end

% build position increment dp=[dx;dy], from control signal dx
dp = [dx;0];

if nargout == 1 % No Jacobians requested
    
    to = fromFrame(r, dp);

else %  Jacobians requested
    
    [to, TOr, TOdt] = fromFrame(r, dp);
    AOa = 1;
    AOda = 1;
    
    ROr = [TOr ; 0 0 AOa];
    ROn = [TOdt(:,1) zeros(2,1) ; 0 AOda];
end
ro = [to;ao];