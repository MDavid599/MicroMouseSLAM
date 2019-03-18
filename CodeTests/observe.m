function [y, Yr, Yp] = observe(r, p)
%   OBSERVE Transform a point P to robot frame and take a
%   range−and−bearing measurement.
%
%   In:
%       r :     robot frame             r = [rx ; ry ; ralpha]
%       p :     point in global frame   p = [px ; py]
%   Out:
%       y:      range−and−bearing measurement
%       Yr:    Jacobian wrt r
%       Yp:    Jacobian wrt p
%   (c) 2010, 2011, 2012 Joan Sola

if nargout == 1 % No Jacobians requested
    
    y = scan(toFrame(r,p));
    
else %  Jacobians requested
    
    [pr, PRr, PRp] = toFrame(r, p);
    [y, Ypr] = scan(pr);
    
    % The chain rule!
    Yr = Ypr*PRr;
    Yp = Ypr*PRp;
end