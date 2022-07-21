%% Analyse Winch Control Logs
%
clear all;
close all;
clc;
%
% Some definitions
Rate = 50; % [Hz]
MinVel = 11.0; % [km/h]
MaxVel = 40; % [km/h]
%% Load log file
opts = delimitedTextImportOptions("NumVariables", 31);
%
% Specify range and delimiter
opts.DataLines = [2, Inf]; % Skip header
opts.Delimiter = ";";
opts.Encoding = "UTF-8";
%
% Specify column names and types
opts.VariableNamingRule = "modify";
opts.VariableNames = ["Time_ms", "dt_ms",  "LoopCounter", "CurrentEncoderReading", "AngularIncrementRaw_deg", "AngularIncrement_deg", "AngularIncrementFiltered_deg", "RevolutionCounter", "RopeVelocity_kmh", "RopeLength_m", "WinchState", "DesiredVelocity_kmh", "CommandedVelocity_kmh", "CurrentError_kmh", "IntegralError_kmh", "DifferentialError_kmh", "ProportionalComponent", "IntegralComponent", "DifferentialComponent", "FeedForwardComponent", "ThrottleServoSetpoint", "ThrottleServoMicroseconds_us", "BreakServoSetpoint", "BreakServoMicroseconds_us", "AccX_g", "AccY_g", "AccZ_g", "norm(Acc)^2_g^2", "EngineState", "EngineVibrationCounter", "ProcessingTime_ms"];
opts.VariableTypes = ["double",  "double", "int32",       "double",                "double",                  "double",               "double",                       "double",            "double",            "double",      "int8",       "double",              "double",                "double",           "double",            "double",                "double",                "double",            "double",                "double",               "double",                "int16",                        "double",             "int16",                     "double", "double", "double", "double",          "int8",        "int16",                  "double"           ];
%
% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "skip";
%
% Import the data
[file, path] = uigetfile('*.log.csv');
WinchData = readtable([path filesep file], opts);
%
% Clear temporary variables
clear opts
%% Some preprocessing
%
% Increase number of winch state to avoid negative states (configuration)
WinchData.WinchState = WinchData.WinchState + 1;
WinchData.RopeVelocity_kmh = -WinchData.RopeVelocity_kmh;
%% Plot timing
figure("Name", "Timing");
subplot(2, 1, 1);
hold on;
plot(WinchData.Time_ms / 1000, WinchData.dt_ms, 'b.');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('\Delta t [ms]');

subplot(2, 1, 2);
hold on;
idx = WinchData.ProcessingTime_ms <= 1 / Rate * 1000;
plot(WinchData.Time_ms(idx) / 1000, WinchData.ProcessingTime_ms(idx), 'g.');
plot(WinchData.Time_ms(~idx) / 1000, WinchData.ProcessingTime_ms(~idx), 'r.');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Processing Time [ms]');
%% Plot Velocities
figure("Name", "Velocities");
subplot(3, 1, 1);
hold on;
plot(WinchData.Time_ms / 1000, WinchData.ThrottleServoSetpoint * max(abs([WinchData.RopeVelocity_kmh; WinchData.CommandedVelocity_kmh; WinchData.DesiredVelocity_kmh])), 'm');
plot(WinchData.Time_ms / 1000, abs(WinchData.DesiredVelocity_kmh), 'g');
plot(WinchData.Time_ms / 1000, abs(WinchData.CommandedVelocity_kmh), 'b');
plot(WinchData.Time_ms / 1000, abs(WinchData.RopeVelocity_kmh), 'r');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Velocity [km/h]');
legend('Setpoint', 'Desired Velocity [km/h]', 'Commanded Velocity [km/h]', 'Current Velocity [km/h]');
subplot(3, 1, 2);
hold on;
plot(WinchData.Time_ms / 1000, WinchData.ThrottleServoSetpoint * max(abs(WinchData.DesiredVelocity_kmh)), 'm');
plot(WinchData.Time_ms / 1000, WinchData.CurrentError_kmh, 'r');
plot(WinchData.Time_ms / 1000, WinchData.IntegralError_kmh, 'g');
plot(WinchData.Time_ms / 1000, WinchData.DifferentialError_kmh, 'b');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Error [km/h]');
legend('Setpoint', 'Current Error [km/h]', 'Integral Error [km/h]', 'Differential Error [km/h]');
subplot(3, 1, 3);
hold on;
plot(WinchData.Time_ms / 1000, WinchData.ThrottleServoSetpoint, 'm');
plot(WinchData.Time_ms / 1000, WinchData.FeedForwardComponent, 'c');
plot(WinchData.Time_ms / 1000, WinchData.ProportionalComponent, 'r');
plot(WinchData.Time_ms / 1000, WinchData.IntegralComponent, 'g');
plot(WinchData.Time_ms / 1000, WinchData.DifferentialComponent, 'b');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Setpoint [0, 1]');
legend('Setpoint', 'Feedforward Component', 'Proportional Component', 'Integral Component', 'Differential Component');
%% Plot Acceleration
figure("Name", "Acceleration");
hold on;
plot(WinchData.Time_ms(2 : end) / 1000, (WinchData.RopeVelocity_kmh(2 : end) - WinchData.RopeVelocity_kmh(1 : end - 1)) * (1 / Rate * 1000) / 3.6, 'r');
plot(WinchData.Time_ms / 1000, WinchData.RopeVelocity_kmh, 'b');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
%% Plot Rope Length
figure("Name", "Rope Length");
hold on;
plot(WinchData.Time_ms / 1000, WinchData.RopeLength_m, 'g');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Rope Length [m]');
%% Plot Angular Increment
figure("Name", "Angular Increment");
hold on;
plot(WinchData.Time_ms / 1000, abs(WinchData.AngularIncrementRaw_deg), 'r');
plot(WinchData.Time_ms / 1000, abs(WinchData.AngularIncrement_deg), 'b');
plot(WinchData.Time_ms / 1000, abs(WinchData.AngularIncrementFiltered_deg), 'g');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Angular Increment [deg]');
%% Plot Engine State Estimator
figure("Name", "Engine State Estimator");
hold on;
plot(WinchData.Time_ms / 1000, sqrt(WinchData.norm_Acc__2_g_2), 'g');
plot(WinchData.Time_ms / 1000, WinchData.EngineVibrationCounter, 'b');
plot(WinchData.Time_ms / 1000, WinchData.EngineState * 100, 'r');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
legend('norm(Acc) [g]', 'EngineVibrationCounter', 'EngineState [0, 100]');
%% Plot Velocity of Throttle
% figure("Name", "Velocities");
% subplot(2, 1, 1);
% hold on;
% s = 1609;
% n = 1782;
% idx = s : length(WinchData.ThrottleServoSetpoint) - n;
% plot(WinchData.ThrottleServoSetpoint(idx), abs(WinchData.RopeVelocity_kmh(idx)), 'r.');
% expData = [];
% throttleValues = 0 : 0.05 : 1;
% for i = throttleValues
%     idx2 = find(abs(WinchData.ThrottleServoSetpoint(idx) - i) < 0.01);
%     tmp = abs(WinchData.RopeVelocity_kmh(idx));
%     m = median(tmp(idx2));
%     expData = [expData m];
% end
% plot(throttleValues, expData, 'b*-', 'MarkerSize', 2);
% plot(throttleValues, 1.31994 * exp(3.32239 * [throttleValues]), 'g-'); % Exponential fitting
% plot(throttleValues, polyval(polyfit(throttleValues(expData >= MinVel), expData(expData >= MinVel), 1), throttleValues), 'c-'); % linear fitting
% grid on; set(gca,'layer','top');
% xlabel('Throttle');
% ylabel('Velocity [km/h]');
% xlim([0 1]);
% ylim([MinVel MaxVel]);
% subplot(2, 1, 2);
% % And the other way around
% plot(abs(WinchData.RopeVelocity_kmh(idx)), WinchData.ThrottleServoSetpoint(idx), 'r.');
% hold on;
% plot(expData, throttleValues, 'b*-', 'MarkerSize', 2);
% plot(expData, log(expData / 1.31994) / 3.32239, 'g-'); % Exponential fitting
% plot(expData, polyval(polyfit(expData(expData >= MinVel), throttleValues(expData >= MinVel), 1), expData), 'c-'); % linear fitting
% grid on; set(gca,'layer','top');
% ylabel('Throttle');
% xlabel('Velocity [km/h]');
% ylim([0 1]);
% xlim([MinVel MaxVel]);
%% Utility functions
function addBackground(WinchData, y)
    hold on;
    [X, Y] = meshgrid(WinchData.Time_ms / 1000, y);
    pcolor(X, Y, repmat((2 * WinchData.WinchState + WinchData.EngineState)', size(X, 1), 1));
    shading flat;
    map = [     0    1.0000    1.0000;  ... % CONFIGURATION     + Engine on
                0    1.0000    1.0000;  ... % CONFIGURATION     + Engine off
                0    0.5000    1.0000;  ... % STANDBY           + Engine off
                0         0    1.0000;  ... % STANDBY           + Engine on
           1.0000    1.0000         0;  ... % SPOOL_UP_REJECTED + Engine off
           1.0000    1.0000         0;  ... % SPOOL_UP_REJECTED + Engine on
           1.0000    1.0000         0;  ... % SPOOL_UP_WARNING  + Engine off
           1.0000    1.0000         0;  ... % SPOOL_UP_WARNING  + Engine on
           1.0000    0.5000         0;  ... % SPOOL_UP          + Engine off
           1.0000    0.5000         0;  ... % SPOOL_UP          + Engine on
           0.5000         0         0; ...  % SHREDDING         + Engine off
           0.5000         0         0; ...  % SHREDDING         + Engine on
           1.0000         0         0; ...  % SPOOL_DOWN        + Engine off
           1.0000         0         0; ...  % SPOOL_DOWN        + Engine on
    ];
    colormap(min(map, 1));
    cb = colorbar;
    cb.Limits = [0 13];
    cb.Ticks = [0 1 2 3 4 5 6 7 8 9 10 11 12 13 14];
    cb.TickLabels = {'CONFIGURATION & Engine OFF', ...
                     'CONFIGURATION & Engine ON', ...
                     'STANDBY & Engine OFF', ...
                     'STANDBY & Engine ON', ...
                     'SPOOL UP REJECTED & Engine OFF', ...
                     'SPOOL UP REJECTED & Engine ON', ...
                     'SPOOL UP WARNING & Engine OFF', ...
                     'SPOOL UP WARNING & Engine ON', ...
                     'SPOOL UP & Engine OFF', ...
                     'SPOOL UP & Engine ON', ...
                     'SHREDDING & Engine OFF', ...
                     'SHREDDING & Engine ON', ...
                     'SPOOL DOWN & Engine OFF', ...
                     'SPOOL DOWN & Engine ON', ...
    };
end