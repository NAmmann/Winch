close all;
clear all;
clc;

calAngle  = 30.2; % deg
% leverArm = 38.1; % mm = 1.5 inch
% leverArm = 31.75; % mm = 1.25 inch
leverArm = 25.4; % mm = 1 inch
O = [28.3; 55.0]; % mm
%
% Calculate travel based on lever arm from 0 to 180 degrees
angle = calAngle : 0.1 : 180 - calAngle; % deg
X = sqrt(sum((repmat(O, 1, length(angle)) - leverArm * [sin(DEG2RAD(angle)); cos(DEG2RAD(angle))]).^2));
%
% Get minimal travel
[minTravel, minIndex] = min(X);
%
% Set minimal travel to zero
X = X - minTravel;
%
% Get maximal travel
[maxTravel, maxIndex] = max(X);
%
% Get angle for minimal travel
minAngle = angle(minIndex);
maxAngle = angle(maxIndex);
maxTravel = maxTravel
%
% Trim data
X = X(minIndex : maxIndex);
angle = angle(minIndex : maxIndex);
%
% Plot data
figure;
plot(angle, X)
xlabel('Angle [deg]');
ylabel('Travel [mm]');
%
% Plot inverse of data
figure;
plot(X, angle)
ylabel('Angle [deg]');
xlabel('Travel [mm]');
%
% Fit ploygon to inverse of data
% y = p1*x^5 + p2*x^4 +
%     p3*x^3 + p4*x^2 +
%     p5*x^1 + p6*x^0
% 
% Coefficients:
%   p1 = 3.5661e-06
%   p2 = -0.00041742
%   p3 = 0.018798
%   p4 = -0.39862
%   p5 = 6.2021
%   p6 = 33.2817
