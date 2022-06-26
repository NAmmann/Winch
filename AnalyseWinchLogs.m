%% Analyse Winch Control Logs
%
clear all;
close all;
clc;
%
% Some definitions
Rate = 50; % [Hz]
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
opts.VariableTypes = ["int64",   "int8",   "int32",       "int16",                 "double",                  "double",               "double",                       "double",            "double",            "double",      "int8",       "double",              "double",                "double",           "double",            "double",                "double",                "double",            "double",                "double",               "double",                "int16",                        "double",             "int16",                     "double", "double", "double", "double",          "int8",        "int16",                  "int8"];
%
% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "skip";
%
% Import the data
WinchData = readtable("Winch.log.csv", opts);
%
% Clear temporary variables
clear opts
%% Some preprocessing
%
% Increase number of winch state to avoid negative states (configuration)
WinchData.winchState = WinchData.winchState + 1;
WinchData.ropeVelocity_kmh = -WinchData.ropeVelocity_kmh;
WinchData.ropeVelocity_ms  = -WinchData.ropeVelocity_ms;
%% Plot timing
figure("Name", "Timing");
subplot(2, 1, 1);
addBackground(WinchData, WinchData.dt_ms)
plot(WinchData.time_ms / 1000, WinchData.dt_ms, 'b.');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('\Delta t [ms]');

subplot(2, 1, 2);
hold on;
addBackground(WinchData, WinchData.processingTime_ms);
idx = WinchData.processingTime_ms <= 1 / Rate * 1000;
plot(WinchData.time_ms(idx) / 1000, WinchData.processingTime_ms(idx), 'g.');
plot(WinchData.time_ms(~idx) / 1000, WinchData.processingTime_ms(~idx), 'r.');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Processing Time [ms]');
%% Plot Velocities
figure("Name", "Velocities");
addBackground(WinchData, WinchData.commandedVelocity_ms * 3.6);
plot(WinchData.time_ms / 1000, WinchData.desiredVelocity_ms * 3.6, 'g');
plot(WinchData.time_ms / 1000, WinchData.commandedVelocity_ms * 3.6, 'b');
plot(WinchData.time_ms / 1000, WinchData.ropeVelocity_ms * 3.6, 'r');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Velocity [km/h]');
%% Plot Acceleration
figure("Name", "Acceleration");
hold on;
plot(WinchData.time_ms(2 : end) / 1000, (WinchData.ropeVelocity_ms(2 : end) - WinchData.ropeVelocity_ms(1 : end - 1)) * 20, 'r');
plot(WinchData.time_ms / 1000, WinchData.ropeVelocity_ms * 3.6, 'b');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
%% Plot Rope Length
figure("Name", "Rope Length");
addBackground(WinchData, WinchData.ropeLength_m);
plot(WinchData.time_ms / 1000, WinchData.ropeLength_m, 'g');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Rope Length [m]');
%% Plot Angular Increment
figure("Name", "Angular Increment");
hold on;
plot(WinchData.time_ms / 1000, WinchData.angularIncrement_deg, 'g');
plot(WinchData.time_ms / 1000, WinchData.angularIncrementRaw_deg, 'b');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
ylabel('Angular Increment [deg]');
%% Plot Engine State Estimator
figure("Name", "Angular Increment");
hold on;
plot(WinchData.time_ms / 1000, WinchData.normAcc2_g2, 'g');
plot(WinchData.time_ms / 1000, WinchData.engineVibrationCounter, 'b');
plot(WinchData.time_ms / 1000, WinchData.engineState, 'r');
grid on; set(gca,'layer','top');
xlabel('Time [s]');
%% Utility functions
function addBackground(WinchData, y)
    hold on;
    [X, Y] = meshgrid(WinchData.time_ms / 1000, y);
    pcolor(X, Y, repmat((2 * WinchData.winchState + WinchData.engineState)', size(X, 1), 1));
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