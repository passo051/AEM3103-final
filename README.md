  # Paper Airplane Numerical Study
  Final Project: AEM 3103 Spring 2024

  - By: Reece Passons

  ## Summary of Findings
 ||Velocity|Flight Path Angle|
 |---|---|---|
 |Nominal|3.55|-0.18|
 |Minimum|2.00|-0.50|
 |Maximum|7.50|0.40|

  Summarized what was accomplished in this study.  Describe 2-4 observations from simulating the flight path.
  Reference the figures below as needed.
  This study expanded on the existing code of Example 1.3-1 Paper Airplane Flight Path, by Robert Stengel. The study found and plotted (Figure 1) the flight trajectories from varying velocity, and from varying the flight path angle (gamma). Next the study found and plotted 100 trajectories from randomly generated velocity and gamma's, it then found the average trajectory and plotted it on the same plot (Figure 2). Lastly, the study found the derivative of height with respect to time, and derivative of range with respect to time of the average trajectory, and plotted these as well (Figure 3). Some observations I had from simulating the flight path include that the plane would almost always return to a trim (steady) state, after 10 to 15 seconds. Additionally, the plane would rarely do a loop motion (depending on the initial velocity and gamma). Lastly, I noticed that the graph of derivatives (Figure 3) corrisponded well to the graph of the trajectories (Figure 2), which makes sense, as the plane initally flies up the change in height over time should be larger.

 
  # Code Listing
  A list of each function/script and a single-line description of what it does.  The name of the function/script should link to the file in the repository on GitHub.
  
This function calculates the equations of motion for the plane, from a given x (Velocity,Gamma,Height,and Range):
```
    function xdot = EqMotion(t,x)
%	Fourth-Order Equations of Aircraft Motion

	global CL CD S m g rho
	
	V 	=	x(1);
	Gam	=	x(2);
	q	=	0.5 * rho * V^2;	% Dynamic Pressure, N/m^2
	
	xdot	=	[(-CD * q * S - m * g * sin(Gam)) / m
				 (CL * q * S - m * g * cos(Gam)) / (m * V)
				 V * sin(Gam)
				 V * cos(Gam)];
end
```
This function calculates derivative of y values with respect to x, using the central difference method of differentiation:
```
function [fp_num] = num_der_central(x, y)
  drl = length(x);
  fp_num = nan(1, drl); % NaN - means Not a Number (An array of size 1 by drl)
  
      for i = 2:drl-1
          fp_num(i) = (y(i+1) - y(i-1))/(x(i+1) - x(i-1));
      
      end
  
  % Handle last element
  fp_num(drl) = (y(drl) - y(drl-1))/(x(drl) - x(drl-1));
  % Backward Difference
  fp_num(i) = (y(drl) - y(drl-1))/(x(drl) - x(drl-1));

end
```
The following is the full script "PaperPlane.m", which does all as described in the original summary of what was accomplished in this study:
```
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

title ('Changing Velocity'), grid on
xlabel('Range, m'), ylabel('Height, m')

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

title('Changing Gamma'), grid on
xlabel('Range, m'), ylabel('Height, m')

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
title('100 Randomly Generated Trajectories'), grid on

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
    'k', 'LineWidth', 3)
xlim([0 25])
hold off

%% Prob 5
% Calculating Derivative values
RangeDerivative = num_der_central(tr, polyval(AverageRange, tr));
HeightDerivative = num_der_central(tr, polyval(AverageHeight, tr));

% Plotting the Derivatives
figure() 
subplot(2,1,1)
plot(tr, HeightDerivative)
title('Derivative of Height w.r.t. Time'), grid on
xlabel('Time s'), ylabel('d(Height), m') 

subplot(2,1,2)
plot(tr, RangeDerivative)
title('Derivative of Range w.r.t. Time'), grid on
xlabel('Time, s'), ylabel('d(Range), m');

%% ADD LEGENDS TO GRAPHS!
disp('ADD LEGENDS TO GRAPHS')
```
  # Figures

  ## Fig. 1: Single Parameter Variation
  <2D trajectory simulated by varying single parameter at at time>
![Single Parameter Variation](https://github.com/passo051/AEM3103-final/assets/167140449/3a17f841-ae96-4f6f-bb76-9fb5412aab56)

In this figure, the velocity and flight path angle is varied based on the values in the table. The black shows the trajectory generated from a nominal  velocity and gamma, the green shows the trajectory generated from a higher velocity and gamma, and the red shows the trajectory generated from a lower velocity and gamma.

  ## Fig. 2: Monte Carlo Simulation
  <2D trajectories simulated using random sampling of parameters, overlay polynomial fit onto plot.>
![Monte Carlo Simulation](https://github.com/passo051/AEM3103-final/assets/167140449/887aa66b-f06c-4fab-8416-f12ce1a787cb)

In this figure, 100 trajectories from randomly generated velocity and gamma's are plotted, as well as the average trajectory in black found from a 9th degree polynomial fit to the data.

 ## Fig. 3: Time Derivatives
 <Time-derivative of height and range for the fitted trajectory>
 
   ![Time Derivative](https://github.com/passo051/AEM3103-final/assets/167140449/b7ad584a-5771-45ed-85a1-b8f179a9cb98)   

  In this figure, the derivative of the average trajectory for height and range with respect to time are shown.

  
