% I. INITIALIZE

% I.1 SIMULATOR−−use capital letters for variable names
%   W: set of external landmarks
W = cloister(-4,4,-4,4,20);    % Type 'help cloister' for help
%   N: number of landmarks
N = size(W,2);
%   R: robot pose [x ; y ; alpha]
R = [0;-2;0];
%   U: control [dx ; dalpha]
U = [0.1 ; 0.05]; % fixing advance and turn increments creates a circle
%   Y: measurements of all landmarks
Y = zeros(2, N);

% I.2 ESTIMATOR
% Map: Gaussian{x,P}
%   x: state vector's mean
x = zeros(numel(R)+numel(W), 1);
%   P: state vector's covariances matrix
P = zeros(numel(x),numel(x));

% System noise: Gaussian{0,Q}
q = [.01;.02];      % amplitude or standard deviation
Q = diag(q.^2);     % covariances matrix

% Measurement noise: Gaussian{0,S}
s = [.1;1*pi/180];  % amplitude or standard deviation
S = diag(s.^2);     % covariances matrix

% Map management
mapspace = false(1,numel(x));   % See README Note #10.
% Landmarks management
landmarks = zeros(2, N);        % See README Note #11.

% Place robot in map
r           = find(mapspace==false, numel(R) ); % set robot pointer
mapspace(r) = true; % block map positions
x(r)        = R;    % initialize robot states
P(r,r)      = 0;    % initialize robot covariance

% I.3 GRAPHICS−−use the variable names of simulated and estimated
%   variables, followed by a capital G to indicate 'graphics'.
%   NOTE: the graphics code is long but absolutely necessary.

% Set figure and axes for Map
mapFig = figure(1); % create figure
cla                 % clear axes
axis([-6 6 -6 6])   % set axes limits
axis square         % set 1:1 aspect ratio

% Simulated World−−set of all landmarks, red crosses
WG = line(...
	'linestyle','none',...
	'marker','+',...
	'color','r',...
	'xdata',W(1,:),...
	'ydata',W(2,:));

% Simulated robot, red triangle
Rshape0 = .2*[...
	2 -1 -1  2; ...
	0  1 -1  0];                 % a triangle at the origin
Rshape = fromFrame(R, Rshape0);  % a triangle at the robot pose
RG = line(...
	'linestyle','-',...
	'marker','none',...
	'color','r',...
	'xdata',Rshape(1,:),...
	'ydata',Rshape(2,:));
% Estimated robot, blue triangle
rG = line(...
	'linestyle','-',...
	'marker','none',...
	'color','b',...
	'xdata',Rshape(1,:),...
	'ydata',Rshape(2,:));

% Estimated robot ellipse, magenta
	reG = line(...
	'linestyle','-',...
	'marker','none',...
	'color','m',...
	'xdata',[ ],...
	'ydata',[ ]);

% Estimated landmark means, blue crosses
lG = line(...
	'linestyle','none',...
	'marker','+',...
	'color','b',...
	'xdata',[ ],...
	'ydata',[ ]);

% Estimated landmark ellipses, green
leG = zeros(1,N);
for i = 1:numel(leG)
leG(i) = line(...
	'linestyle','-',...
	'marker','none',...
	'color','g',...
	'xdata',[ ],...
	'ydata',[ ]);
end

% II. TEMPORAL LOOP

for t = 1:200
	
	% II.1 SIMULATOR
	% a. motion
	n = q .*randn(2,1);         	% perturbation vector
	R = move(R, U, zeros(2,1) ); 	% we will perturb the estimator
					% instead of the simulator

	% b. observations
    for i = 1:N                    	% i: landmark index
        v      = s .*randn(2,1);  	% measurement noise
        Y(:,i) = observe(R, W(:,i)) + v;
    end
    
	% II.2 ESTIMATOR
	% a. create dynamic map pointers to be used hereafter
	m  = landmarks(landmarks~=0)'; 	% all pointers to landmarks
	rm = [r , m];                  	% all used states: robot and landmarks
					% ( also OK is  rm = find(mapspace); )

	% b. Prediction−−robot motion
	[x(r), Rr, Rn] = move(x(r), U, n); 	% Estimator perturbed with n
	P(r,m)           = Rr*P(r,m);  		% See PDF notes 'SLAM course.pdf'
	P(m,r)           = P(r,m)';
	P(r,r)           = Rr*P(r,r)*Rr' + Rn*Q*Rn';

	% c. Landmark correction−−known landmarks
	lids = find( landmarks(1,:) ); % returns all indices of existing landmarks
	for i = lids

		% expectation: Gaussian{e,E}
		l             = landmarks(:, i)';    	% landmark pointer
		[e, Er, El] = observe(x(r), x(l) ); 	% this is h(x) in EKF
		rl            = [r , l];              	% pointers to robot and lmk.
		E_rl          = [Er , El];          	% expectation Jacobian
		E             = E_rl*P(rl, rl)*E_rl';

		% measurement of landmark i
		Yi = Y(:, i);
		% innovation: Gaussian{z,Z}
		z = Yi-e; 				% this is z = y−h(x) in EKF
		% we need values around zero for angles:
		if z(2)>pi
			z(2) = z(2)-2*pi;
		end
		if z(2)< -pi
			z(2) = z(2) + 2*pi;
		end
		Z = S + E;

		% Individual compatibility check at Mahalanobis distance of 3−sigma
		% (See appendix of documentation file 'SLAM course.pdf')
		if z'*Z^-1*z<9

			% Kalman gain
			K = P(rm, rl)*E_rl'*Z^-1; % this is K = P*H'*Zˆ−1 in EKF

			% map update (use pointer rm)
			x(rm)    = x(rm)    + K*z;
			P(rm,rm) = P(rm,rm)-K*Z*K';
	end
end

	% d. Landmark Initialization−−one new landmark only at each iteration
	lids = find(landmarks(1,:)==0);    % all non−initialized landmarks
	if~isempty(lids)                  % there are still landmarks to initialize
		i = lids(randi(numel(lids)));  % pick one landmark randomly, its index is i
		l = find(mapspace==false, 2);  % pointer of the new landmark in the map
		if~isempty(l)                 % there is still space in the map
		mapspace(l)    = true;     % block map space
		landmarks(:,i) = l;        % store landmark pointers

		% measurement
        Yi = Y(:,i);

		% initialization
		[x(l), Lr, Ly] = invObserve(x(r), Yi);
		P(l,rm)          = Lr*P(r,rm);
		P(rm,l)          = P(l,rm)';
		P(l,l)           = Lr*P(r,r)*Lr' + Ly*S*Ly';
		end
	end

	% II.3 GRAPHICS

	% Simulated robot

	Rshape = fromFrame(R, Rshape0);
	set(RG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));

	% Estimated robot
	Rshape = fromFrame(x(r), Rshape0);
	set(rG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));

	% Estimated robot ellipse
	re      = x(r(1:2));            	% robot position mean
	RE      = P(r(1:2),r(1:2));     	% robot position covariance
	[xx,yy] = cov2elli(re,RE,3,16); 	% x−and y−coordinates of contour
	set(reG, 'xdata', xx, 'ydata', yy);

	% Estimated landmarks
	lids = find(landmarks(1,:));	 	% all indices of mapped landmarks
	lx   = x(landmarks(1,lids)); % all x−coordinates
	ly   = x(landmarks(2,lids)); % all y−coordinates
	set(lG, 'xdata', lx, 'ydata', ly);

	% Estimated landmark ellipses−−one per landmark
	for i = lids
		l       = landmarks(:,i);
		le      = x(l);
		LE      = P(l,l);
		[xx,yy] = cov2elli(le,LE,3,16);
		set(leG(i), 'xdata', xx, 'ydata', yy);
	end

	% force Matlab to draw all graphic objects before next iteration
	drawnow
%     pause(1)
end
