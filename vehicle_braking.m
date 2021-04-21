%% Vehicle braking to a stop
% Animation of a road vehicle braking to a stop with position, speed,
% acceleration and braking force plots.
%
%% 

clear ; close all ; clc

%% Scenario

% Road
road_Width              = 10;       % Road width                    [m]
road_Margin             = 2;        % Road margin                   [m]

% Vehicle
vehicle_Length          = 4.65;     % Length of the vehicle         [m]
vehicle_Width           = 1.78;     % Width of the vehicle          [m]
vehicle_Initial_Speed   = 72/3.6;   % Initial speed of the vehicle  [m/s]
vehicle_Mass            = 1145;     % Mass of the vehicle           [kg]
vehicle_Area            = 2.5;      % Frontal area of the vehicle   [m2]
vehicle_Cd              = 0.35;     % Drag coefficient              [-]
air_Density             = 1;        % Air density                   [kg/m3]

% Lumped air drag coefficient [N(s/m)2]
C = 0.5 * vehicle_Area * vehicle_Cd * air_Density;   

% Vehicle struct
vehicle.C = C;
vehicle.M = vehicle_Mass;

% Parameters
tf      = 100;                      % Final time                    [s]
% OBS: tf must be larger than the stopping time.
fR      = 30;                       % Frame rate                    [fps]
dt      = 1/fR;                     % Time resolution               [s]
TSPAN   = linspace(0,tf,tf*fR);     % Time                          [s]

%% Braking force

% Brake
brake_Time_Constant     = 1;        % Brake time constant           [s]
brake_Force_Max         = 5000;     % Brake max. force              [N]
% Braking force during time span [N]
FbBrakingForce          = brake_Force_Max*(1-exp(-brake_Time_Constant*TSPAN));

% Brake struct
brake.time  = TSPAN;
brake.force = FbBrakingForce;

%% Simulation

% Initial conditions [position speed]
Z0 = [0 vehicle_Initial_Speed]; 

% Options:
% Simulation ends when v=0 (See auxiliary function)
options = odeset('events',@vehicleAtRest);
% Integration
[TOUT,ZOUT] = ode45(@(t,z) vehicle_braking_dynamics(t,z,vehicle,brake),TSPAN,Z0,options);

% States
vehicle_position    = ZOUT(:,1);
vehicle_speed       = ZOUT(:,2);
% Acceleration
% Preallocating
vehicle_accel       = zeros(1,length(TOUT));
for i=1:length(TOUT)
    dz = vehicle_braking_dynamics(TOUT(i),ZOUT(i,:),vehicle,brake);
    vehicle_accel(i) = dz(2);
end

%% Animation

figure
set(gcf,'Position',[270   140   640     360  ])

% Create and open video writer object
v = VideoWriter('braking_dynamics.avi');
v.Quality = 100;
open(v);

for i=1:length(TOUT)
    subplot(3,2,1)
        hold on ; grid on
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*max(vehicle_position)])
        cla 
        plot(TOUT,vehicle_position)
        plot([TOUT(i) TOUT(i)],[0 1.2*max(vehicle_position)],'k--') 
        xlabel('Time [s]')
        ylabel('Position [m]')
        title('Position')
    subplot(3,2,2)
        hold on ; grid on
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*max(vehicle_speed)])
        cla 
        plot(TOUT,vehicle_speed)
        plot([TOUT(i) TOUT(i)],[0 1.2*max(vehicle_speed)],'k--') 
        xlabel('Time [s]')
        ylabel('Speed [m/s]')
        title('Speed')
    subplot(3,2,3)
        hold on ; grid on
        set(gca,'xlim',[0 TOUT(end)],'ylim',[1.2*min(vehicle_accel) 1.2*max(vehicle_accel)])
        cla 
        plot(TOUT,vehicle_accel)
        plot([TOUT(i) TOUT(i)],[1.2*min(vehicle_accel) 1.2*max(vehicle_accel)],'k--') 
        xlabel('Time [s]')
        ylabel('Acceleration [m/s2]')
        title('Acceleration')
    subplot(3,2,4)
        hold on ; grid on
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*max(FbBrakingForce)])
        cla 
        plot(TSPAN,FbBrakingForce)
        plot([TOUT(i) TOUT(i)],[0 1.2*max(FbBrakingForce)],'k--') 
        xlabel('Time [s]')
        ylabel('Braking force [N]')
        title('Braking force')
    subplot(3,2,5:6)
        hold on ; axis equal
        cla 
        % Position of the vehicle instant [m]
        vehicle_position_inst = vehicle_position(i);
        road_Length           = 1.2*max(vehicle_position); % Road length [m]
        sideMarkingsX         = [-1.5*vehicle_Length road_Length];
        
        set(gca,'xlim',sideMarkingsX,'ylim',[-road_Width/2-road_Margin +road_Width/2+road_Margin])

        plot(sideMarkingsX,[+road_Width/2 +road_Width/2],'k--') % Left marking
        plot(sideMarkingsX,[-road_Width/2 -road_Width/2],'k--') % Right marking

        % Dimensions
        vehicle_dimension_X = [vehicle_position_inst vehicle_position_inst vehicle_position_inst-vehicle_Length vehicle_position_inst-vehicle_Length];
        vehicle_dimension_Y = [+vehicle_Width/2 -vehicle_Width/2 -vehicle_Width/2 +vehicle_Width/2];
        % Plotting
        fill(vehicle_dimension_X,vehicle_dimension_Y,'r')
        
        xlabel('Lon. distance [m]')
        ylabel('Lat. distance [m]')
        
    frame = getframe(gcf);
    writeVideo(v,frame);
end 

close(v);

%% Auxiliary functions

function dz = vehicle_braking_dynamics(t,z,vehicle,brake)

    % States
    % z1 = z(1);
    z2 = z(2);
    
    % Parameters
    C = vehicle.C;
    M = vehicle.M;

    % Brake force
    timeBraking  = brake.time;
    forceBraking = brake.force;
    Fb = interp1(timeBraking,forceBraking,t);
    
    % State Equations
    dz(1,1) = z2;
    dz(2,1) = -(Fb + C*z2^2)/M;

end

function [speed,isterminal,direction] = vehicleAtRest(~,z)
    speed       = z(2); 
    isterminal  = 1;    
    direction   = 0;    
end