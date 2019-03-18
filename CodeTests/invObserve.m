function [p, Pr, Py] = invObserve(r, y)
%   INVOBSERVE Backproject a range−and−bearing measurement and transform
%   to map frame.
%
%   In:
%       r :     robot frame     r = [rx ; ry ; ralpha]
%       y :     measurement     y = [range ; bearing]
%   Out:
%       p :     point in sensor frame
%       Pr:    Jacobian wrt r
%       Py:    Jacobian wrt y
%   (c) 2010, 2011, 2012 Joan Sola
if nargout == 1 % No Jacobians requested
    p   = fromFrame(r, invScan(y));
else %  Jacobians requested
    [pr, PRy]    = invScan(y);
    [p, Pr, Ppr] = fromFrame(r, pr);
    
    % here the chain rule !
    Py = Ppr*PRy;
end
end
function f()
%% Symbolic code below−−Generation and/or test of Jacobians
%−Enable 'cell mode' to use this section
%−Left−click once on the code below−the cell should turn yellow
%−Type ctrl+enter (Windows, Linux) or Cmd+enter (MacOSX) to execute
%−Check the Jacobian results in the Command Window.
syms rx ry ra yd ya real
r = [rx;ry;ra];
y = [yd;ya];
[p, Pr, Py] = invObserve(r, y); % We extract also the coded Jacobians Pr and Py
% We use the symbolic result to test the coded Jacobians
simplify(Pr-jacobian(p,r))  % zero−matrix if coded Jacobian is correct
simplify(Py-jacobian(p,y))  % zero−matrix if coded Jacobian is correct
end