%% Comparison of braking performance
% Simulation and animation of two vehicles braking to a stop with position,
% speed, acceleration and braking force plots.
%
%% 

clear ; close all ; clc

%% Scenario

% Road
road_Width              = 10;       % Road width                    [m]
road_Margin             = 2;        % Road margin                   [m]
air_Density             = 1;        % Air density                   [kg/m3]

% Vehicle 1
vehicle_1_Length        = 4.65;     % Length of the vehicle         [m]
vehicle_1_Width         = 1.78;     % Width of the vehicle          [m]
vehicle_1_Init_Speed    = 72/3.6;   % Initial speed of the vehicle  [m/s]
vehicle_1_Mass          = 1145;     % Mass of the vehicle           [kg]
vehicle_1_Area          = 2.5;      % Frontal area of the vehicle   [m2]
vehicle_1_Cd            = 0.35;     % Drag coefficient              [-]
vehicle_1_Lat_Pos       = 2.5;      % Lateral position              [m]
% Lumped air drag coefficient [N(s/m)2]
vehicle_1_C = 0.5 * vehicle_1_Area * vehicle_1_Cd * air_Density; 
% Vehicle 1 struct
vehicle_1.C = vehicle_1_C;
vehicle_1.M = vehicle_1_Mass;

% Vehicle 2
vehicle_2_Length        = 4.65;     % Length of the vehicle         [m]
vehicle_2_Width         = 1.78;     % Width of the vehicle          [m]
vehicle_2_Init_Speed    = 72/3.6;   % Initial speed of the vehicle  [m/s]
vehicle_2_Mass          = 1145;     % Mass of the vehicle           [kg]
vehicle_2_Area          = 2.5;      % Frontal area of the vehicle   [m2]
vehicle_2_Cd            = 0.35;     % Drag coefficient              [-]
vehicle_2_Lat_Pos       = -2.5;     % Lateral position              [m]
% Lumped air drag coefficient [N(s/m)2]
vehicle_2_C = 0.5 * vehicle_2_Area * vehicle_2_Cd * air_Density; 
% Vehicle 2 struct
vehicle_2.C = vehicle_2_C;
vehicle_2.M = vehicle_2_Mass;

% Parameters
playback_speed = 1;                  % Speed of playback
tf      = 100;                      % Final time                    [s]
% OBS: tf must be larger than the stopping time.
fR      = 30/playback_speed;                       % Frame rate                    [fps]
dt      = 1/fR;                     % Time resolution               [s]
TSPAN   = linspace(0,tf,tf*fR);     % Time                          [s]

%% Braking force

% Vehicle 1
% Brake
vehicle_1_brake_Time_Constant     = 1;        % Brake time constant           [s]
vehicle_1_brake_Force_Max         = 3000;     % Brake max. force              [N]
% Braking force during time span [N]
vehicle_1_FbBrakingForce          = vehicle_1_brake_Force_Max*(1-exp(-vehicle_1_brake_Time_Constant*TSPAN));
% Brake struct
vehicle_1_brake.time  = TSPAN;
vehicle_1_brake.force = vehicle_1_FbBrakingForce;

% Vehicle 2
% Brake
vehicle_2_brake_Time_Constant     = 1;        % Brake time constant           [s]
vehicle_2_brake_Force_Max         = 2000;     % Brake max. force              [N]
% Braking force during time span [N]
vehicle_2_FbBrakingForce          = vehicle_2_brake_Force_Max*(1-exp(-vehicle_2_brake_Time_Constant*TSPAN));
% Brake struct
vehicle_2_brake.time  = TSPAN;
vehicle_2_brake.force = vehicle_2_FbBrakingForce;

%% Simulation

% Initial conditions [position speed]
vehicle_1_Z0 = [0 vehicle_1_Init_Speed]; 
vehicle_2_Z0 = [0 vehicle_2_Init_Speed]; 

% Options:
% Simulation ends when v=0 (See auxiliary function)
options = odeset('events',@vehicleAtRest);
% Integration vehicle 1
[TOUT_1,ZOUT_1] = ode45(@(t,z) vehicle_braking_dynamics(t,z,vehicle_1,vehicle_1_brake),TSPAN,vehicle_1_Z0,options);
% Integration vehicle 2
[TOUT_2,ZOUT_2] = ode45(@(t,z) vehicle_braking_dynamics(t,z,vehicle_2,vehicle_2_brake),TSPAN,vehicle_2_Z0,options);

% Vehicle 1 States 
vehicle_1_position    = ZOUT_1(:,1);
vehicle_1_speed       = ZOUT_1(:,2);
% Vehicle 1 Acceleration
% Vehicle 1 Preallocating
vehicle_1_accel       = zeros(length(TOUT_1),1);
for i=1:length(TOUT_1)
    dz = vehicle_braking_dynamics(TOUT_1(i),ZOUT_1(i,:),vehicle_1,vehicle_1_brake);
    vehicle_1_accel(i) = dz(2);
end

% Vehicle 2 States
vehicle_2_position    = ZOUT_2(:,1);
vehicle_2_speed       = ZOUT_2(:,2);
% Vehicle 2 Acceleration
% Vehicle 2 Preallocating
vehicle_2_accel       = zeros(length(TOUT_2),1);
for i=1:length(TOUT_2)
    dz = vehicle_braking_dynamics(TOUT_2(i),ZOUT_2(i,:),vehicle_2,vehicle_2_brake);
    vehicle_2_accel(i) = dz(2);
end

% Typically, length(TOUT_1) different than length(TOUT_2). Adjusting:
if TOUT_1(end) > TOUT_2(end)
    vehicle_2_position  = [vehicle_2_position   ; vehicle_2_position(end)*ones(length(TOUT_1)-length(TOUT_2),1)];
    vehicle_2_speed     = [vehicle_2_speed      ; vehicle_2_speed(end)*ones(length(TOUT_1)-length(TOUT_2),1)];
    vehicle_2_accel     = [vehicle_2_accel      ; zeros(length(TOUT_1)-length(TOUT_2),1)];
    TOUT_2 = TOUT_1;
elseif TOUT_1(end) < TOUT_2(end)
    vehicle_1_position  = [vehicle_1_position   ; vehicle_1_position(end)*ones(length(TOUT_2)-length(TOUT_1),1)];
    vehicle_1_speed     = [vehicle_1_speed      ; vehicle_1_speed(end)*ones(length(TOUT_2)-length(TOUT_1),1)];
    vehicle_1_accel     = [vehicle_1_accel      ; zeros(length(TOUT_2)-length(TOUT_1),1)];
    TOUT_1 = TOUT_2;
end 

%% Animation

figure
set(gcf,'Position',[50 50 1280 720]) % 720p
% set(gcf,'Position',[50 50 854 480]) % 480p

% Create and open video writer object
v = VideoWriter('Comparison_of_braking_performance.avi');
v.Quality   = 100;
% v.FrameRate = fR;
open(v);

for i=1:length(TOUT_1)
    subplot(3,2,1)
        hold on ; grid on
        set(gca,'xlim',[0 max([TOUT_1(end) TOUT_2(end)])],'ylim',[0 1.2*max([vehicle_1_position(end) vehicle_2_position(end)])])
        cla 
        % Vehicle 1
        plot(TOUT_1,vehicle_1_position,'m')
        % Vehicle 2
        plot(TOUT_2,vehicle_2_position,'c')
        plot([TOUT_1(i) TOUT_1(i)],[0 1.2*max([vehicle_1_position(end) vehicle_2_position(end)])],'k--') 
        xlabel('Time [s]')
        ylabel('Position [m]')
        title('Position')
    subplot(3,2,2)
        hold on ; grid on
        set(gca,'xlim',[0 max([TOUT_1(end) TOUT_2(end)])],'ylim',[0 1.2*max([vehicle_1_speed(1) vehicle_2_speed(1)])])
        cla 
        % Vehicle 1
        plot(TOUT_1,vehicle_1_speed,'m')
        % Vehicle 2
        plot(TOUT_2,vehicle_2_speed,'c')
        plot([TOUT_1(i) TOUT_1(i)],[0 1.2*max([vehicle_1_speed(1) vehicle_2_speed(1)])],'k--') 
        xlabel('Time [s]')
        ylabel('Speed [m/s]')
        title('Speed')
    subplot(3,2,3)
        hold on ; grid on
        set(gca,'xlim',[0 max([TOUT_1(end) TOUT_2(end)])],'ylim',[1.2*min([min(vehicle_1_accel) min(vehicle_2_accel)]) 1.2*max([max(vehicle_1_accel) max(vehicle_2_accel)])])
        cla 
        % Vehicle 1
        plot(TOUT_1,vehicle_1_accel,'m')
        % Vehicle 2
        plot(TOUT_2,vehicle_2_accel,'c')
        plot([TOUT_1(i) TOUT_1(i)],[1.2*min([min(vehicle_1_accel) min(vehicle_2_accel)]) 1.2*max([max(vehicle_1_accel) max(vehicle_2_accel)])],'k--') 
        xlabel('Time [s]')
        ylabel('Acceleration [m/s2]')
        title('Acceleration')
    subplot(3,2,4)
        hold on ; grid on
        set(gca,'xlim',[0 max([TOUT_1(end) TOUT_2(end)])],'ylim',[0 1.2*max([max(vehicle_1_FbBrakingForce) max(vehicle_2_FbBrakingForce)])])
        cla 
        % Vehicle 1
        plot(TSPAN,vehicle_1_FbBrakingForce,'m')
        % Vehicle 2
        plot(TSPAN,vehicle_2_FbBrakingForce,'c')
        plot([TOUT_1(i) TOUT_1(i)],[0 1.2*max([max(vehicle_1_FbBrakingForce) max(vehicle_2_FbBrakingForce)])],'k--') 
        xlabel('Time [s]')
        ylabel('Braking force [N]')
        title('Braking force')
    subplot(3,2,5:6)
        hold on ; axis equal
        cla 
        % Position of the vehicle 1 instant [m]
        vehicle_1_position_inst = vehicle_1_position(i);
        % Position of the vehicle 1 instant [m]
        vehicle_2_position_inst = vehicle_2_position(i);
        
        road_Length           = 1.2*max([max(vehicle_1_position) max(vehicle_2_position)]); % Road length [m]
        sideMarkingsX         = [-1.5*vehicle_1_Length road_Length];
        
        set(gca,'xlim',sideMarkingsX,'ylim',[-road_Width/2-road_Margin +road_Width/2+road_Margin])

        plot(sideMarkingsX,[+road_Width/2 +road_Width/2],'k--') % Left marking
        plot(sideMarkingsX,[0 0],'k--')                         % Center marking
        plot(sideMarkingsX,[-road_Width/2 -road_Width/2],'k--') % Right marking

        % Vehicle 1 Dimensions
        vehicle_1_dimension_X = [vehicle_1_position_inst vehicle_1_position_inst vehicle_1_position_inst-vehicle_1_Length vehicle_1_position_inst-vehicle_1_Length];
        vehicle_1_dimension_Y = [+vehicle_1_Width/2+vehicle_1_Lat_Pos -vehicle_1_Width/2+vehicle_1_Lat_Pos -vehicle_1_Width/2+vehicle_1_Lat_Pos +vehicle_1_Width/2+vehicle_1_Lat_Pos];
        % Vehicle 1 Plotting
        fill(vehicle_1_dimension_X,vehicle_1_dimension_Y,'m')
        
        % Vehicle 2 Dimensions
        vehicle_2_dimension_X = [vehicle_2_position_inst vehicle_2_position_inst vehicle_2_position_inst-vehicle_2_Length vehicle_2_position_inst-vehicle_2_Length];
        vehicle_2_dimension_Y = [+vehicle_2_Width/2+vehicle_2_Lat_Pos -vehicle_2_Width/2+vehicle_2_Lat_Pos -vehicle_2_Width/2+vehicle_2_Lat_Pos +vehicle_2_Width/2+vehicle_2_Lat_Pos];
        % Vehicle 2 Plotting
        fill(vehicle_2_dimension_X,vehicle_2_dimension_Y,'c')
        
        xlabel('Lon. distance [m]')
        ylabel('Lat. distance [m]')
        title(strcat('Time=',num2str(TOUT_1(i),'%.3f'),' s'))
%         title(strcat('Time=',num2str(TOUT_1(i),'%.3f'),' s (Playback speed=',num2str(playback_speed),')'))
        
    frame = getframe(gcf);
    writeVideo(v,frame);
end 

% Additional frames repeating the last frame
% Parameters
tf_add      = 5;                    % Final time                        [s]
fR_add      = 30;                   % Frame rate                        [fps]
dt_add      = 1/fR;                 % Time resolution                   [s]
time_add    = linspace(0,tf_add,tf_add*fR_add); % Time                  [s]
for i=1:length(time_add)
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
