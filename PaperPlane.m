%	Example 1.3-1 Paper Airplane Flight Path
%	Copyright 2005 by Robert Stengel
%	August 23, 2005

	global CL CD S m g rho	
	S		=	0.017;			% Reference Area, m^2
	AR		=	0.86;			% Wing Aspect Ratio
	e		=	0.9;			% Oswald Efficiency Factor;
	m		=	0.003;			% Mass, kg
	g		=	9.8;			% Gravitational acceleration, m/s^2
	rho		=	1.225;			% Air density at Sea Level, kg/m^3	
	CLa		=	3.141592 * AR/(1 + sqrt(1 + (AR / 2)^2));
							% Lift-Coefficient Slope, per rad
	CDo		=	0.02;			% Zero-Lift Drag Coefficient
	epsilon	=	1 / (3.141592 * e * AR);% Induced Drag Factor	
	CL		=	sqrt(CDo / epsilon);	% CL for Maximum Lift/Drag Ratio
	CD		=	CDo + epsilon * CL^2;	% Corresponding CD
	LDmax	=	CL / CD;			% Maximum Lift/Drag Ratio
	Gam		=	-atan(1 / LDmax);	% Corresponding Flight Path Angle, rad
	V		=	sqrt(2 * m * g /(rho * S * (CL * cos(Gam) - CD * sin(Gam))));
							% Corresponding Velocity, m/s
	Alpha	=	CL / CLa;			% Corresponding Angle of Attack, rad
	
%	a) Equilibrium Glide at Maximum Lift/Drag Ratio
	H		=	2;			% Initial Height, m
	R		=	0;			% Initial Range, m
	to		=	0;			% Initial Time, sec
	tf		=	6;			% Final Time, sec
	tspan	=	[to tf];
% 	xo		=	[V;Gam;H;R];
% 	[ta,xa]	=	ode23('EqMotion',tspan,xo);
% 
% %	b) Oscillating Glide due to Zero Initial Flight Path Angle
% 	xo		=	[V;0;H;R];
% 	[tb,xb]	=	ode23('EqMotion',tspan,xo);
% 
% %	c) Effect of Increased Initial Velocity
% 	xo		=	[1.5*V;0;H;R];
% 	[tc,xc]	=	ode23('EqMotion',tspan,xo);
% 
% %	d) Effect of Further Increase in Initial Velocity
% 	xo		=	[3*V;0;H;R];
% 	[td,xd]	=	ode23('EqMotion',tspan,xo);
% 
% 	figure
% 	plot(xa(:,4),xa(:,3),xb(:,4),xb(:,3),xc(:,4),xc(:,3),xd(:,4),xd(:,3))
% 	xlabel('Range, m'), ylabel('Height, m'), grid
% 
% 	figure
% 	subplot(2,2,1)
% 	plot(ta,xa(:,1),tb,xb(:,1),tc,xc(:,1),td,xd(:,1))
% 	xlabel('Time, s'), ylabel('Velocity, m/s'), grid
% 	subplot(2,2,2)
% 	plot(ta,xa(:,2),tb,xb(:,2),tc,xc(:,2),td,xd(:,2))
% 	xlabel('Time, s'), ylabel('Flight Path Angle, rad'), grid
% 	subplot(2,2,3)
% 	plot(ta,xa(:,3),tb,xb(:,3),tc,xc(:,3),td,xd(:,3))
% 	xlabel('Time, s'), ylabel('Altitude, m'), grid
% 	subplot(2,2,4)
% 	plot(ta,xa(:,4),tb,xb(:,4),tc,xc(:,4),td,xd(:,4))
% 	xlabel('Time, s'), ylabel('Range, m'), grid

%% Prob 2

% Nominal Velocity
xo = [3.55;Gam;H;R];
[t1,x1] = ode23('EqMotion',tspan,xo);

% Minimum Velocity
xo = [2;Gam;H;R];
[t2,x2] = ode23('EqMotion',tspan,xo);

% Maximum Velocity
xo = [7.5;Gam;H;R];
[t3,x3] = ode23('EqMotion',tspan,xo);

% Plot of differing velocities
subplot(2,1,1)
plot(x1(:,4),x1(:,3), 'k')
hold on
plot(x2(:,4),x2(:,3), 'r')
plot(x3(:,4),x3(:,3), 'g')
hold off

title ('Changing Velocity'), xlabel('Range, m'), ylabel('Height, m')

% Nominal Gamma
xo = [V;-0.18;H;R];
[t1,x1] = ode23('EqMotion',tspan,xo);

% Minimum Gamma
xo = [V;-0.5;H;R];
[t2,x2] = ode23('EqMotion',tspan,xo);

% Maximum Gamma
xo = [V;0.4;H;R];
[t3,x3] = ode23('EqMotion',tspan,xo);

% Plot of differing gamma's
subplot(2,1,2);
plot(x1(:,4),x1(:,3), 'k')
hold on
plot(x2(:,4),x2(:,3), 'r')
plot(x3(:,4),x3(:,3), 'g')
hold off

title ('Changing Gamma'), xlabel('Range, m'), ylabel('Height, m')

%% Prob 3
% Definine ranges for parameters
Vmin = 2;
Vmax = 7.5;
Gmin = -0.5;
Gmax = 0.4;
tspan = linspace(to, tf, 100);

% Creating trajectory arrays
Ranges = [];
Heights = [];
Times = [];

% Assinging color values for plot
lightBLUE = [0.356862745098039,0.811764705882353,0.956862745098039];
darkBLUE = [0.0196078431372549,0.0745098039215686,0.670588235294118];

% Generating the figure to plot each randomly generated trajectory
figure
xlabel('Range, m'), ylabel('Height, m')
hold on

for i = 1:100
    % Calculating a random velocity and gamma within respective ranges
    Vrand = Vmin + (Vmax-Vmin)*rand(1);
    Grand = Gmin + (Gmax-Gmin)*rand(1);
    
    % Generating random trajectory
    xo = [Vrand;Grand;H;R];
    [tr,xr] = ode23('EqMotion',tspan,xo);
    
    % Storing the time, range, and height for later use
    Times = [Times, tr];
    Ranges = [Ranges, xr(:,4)];
    Heights = [Heights, xr(:,3)];
    
    % Plotting random trajectory
    color = lightBLUE + (darkBLUE-lightBLUE)*((Vrand-Vmin)/(Vmax));
    plot(xr(:,4),xr(:,3), 'Color', color)
end

%% Prob 4
% Assinging the average trajectory
AverageRange = polyfit(Times, Ranges, 9);
AverageHeight = polyfit(Times, Heights, 9);

% Creating a same size array for time
TimeDerivative = linspace(0,6,10);

% Plotting the average trajectory
plot(polyval(AverageRange, tr), polyval(AverageHeight, tr), ...
    'k', 'LineWidth', 3);

%% Prob 5
% Calculating Derivative values
RangeDerivative = num_der_central(tr, polyval(AverageRange, tr));
HeightDerivative = num_der_central(tr, polyval(AverageHeight, tr));

% Plotting the Derivatives
figure() 
subplot(2,1,1)
plot(tr, HeightDerivative)
title('Derivative of Height w.r.t. Time')
xlabel('Time s'), ylabel('d(Height), m') 

subplot(2,1,2)
plot(tr, RangeDerivative)
title('Derivative of Range w.r.t. Time')
xlabel('Time, s'), ylabel('d(Range), m');
