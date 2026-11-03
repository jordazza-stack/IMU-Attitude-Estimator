%% IMU Attitude Estimator — Complementary Filter
% Estimates roll and pitch from gyroscope + accelerometer data.
%
% By default it runs on SIMULATED data with a known "true" attitude, so the
% error of each method can be measured. Set useSimulated = false to load a
% MATLAB Mobile sensor log instead.
%
% Author: Jesus Ordaz, UC Irvine

clear; clc; close all;

%% 1) Settings
useSimulated = true;      % false -> load a phone log from data/
logFile      = 'data/sensorlog.mat';
alpha        = 0.98;      % complementary filter weight on the gyro

%% 2) Get data
% Units: accel in m/s^2, gyro in rad/s, t in seconds
if useSimulated
    rng(0);                          % repeatable noise
    fs = 100;                        % sample rate (Hz)
    t  = (0:1/fs:60)';               % 60 seconds
    g  = 9.81;

    % "True" motion: slow rocking in roll and pitch
    rollTrue  = deg2rad(30) * sin(2*pi*0.10*t);
    pitchTrue = deg2rad(20) * sin(2*pi*0.07*t);

    % Accelerometer sees gravity (plus noise)
    accNoise = 0.3;
    ax = -g*sin(pitchTrue)                + accNoise*randn(size(t));
    ay =  g*cos(pitchTrue).*sin(rollTrue) + accNoise*randn(size(t));
    az =  g*cos(pitchTrue).*cos(rollTrue) + accNoise*randn(size(t));

    % Gyro sees angular rate (plus a constant bias and noise)
    gyroBias = deg2rad(0.5);
    gx = gradient(rollTrue, t)  + gyroBias + 0.01*randn(size(t));
    gy = gradient(pitchTrue, t) + gyroBias + 0.01*randn(size(t));
else
    % MATLAB Mobile log: variables Acceleration and AngularVelocity (timetables)
    S   = load(logFile);
    acc = S.Acceleration;
    gyr = S.AngularVelocity;
    t   = seconds(acc.Timestamp - acc.Timestamp(1));
    ax = acc.X;  ay = acc.Y;  az = acc.Z;
    gx = interp1(seconds(gyr.Timestamp - acc.Timestamp(1)), gyr.X, t, 'linear', 'extrap');
    gy = interp1(seconds(gyr.Timestamp - acc.Timestamp(1)), gyr.Y, t, 'linear', 'extrap');
end

N  = numel(t);
dt = [0; diff(t)];

%% 3) Accelerometer-only tilt (no drift, but noisy)
rollAcc  = atan2(ay, az);
pitchAcc = atan2(-ax, sqrt(ay.^2 + az.^2));

%% 4) Gyro-only integration (smooth, but drifts)
rollGyro  = rollAcc(1)  + cumtrapz(t, gx);
pitchGyro = pitchAcc(1) + cumtrapz(t, gy);

%% 5) Complementary filter
% angle = alpha*(angle + gyro*dt) + (1-alpha)*accel_angle
% Run step by step, like a real flight computer that only knows the past.
roll  = zeros(N,1);  roll(1)  = rollAcc(1);
pitch = zeros(N,1);  pitch(1) = pitchAcc(1);
for k = 2:N
    roll(k)  = alpha*(roll(k-1)  + gx(k)*dt(k)) + (1-alpha)*rollAcc(k);
    pitch(k) = alpha*(pitch(k-1) + gy(k)*dt(k)) + (1-alpha)*pitchAcc(k);
end

%% 6) Plot results
figure('Name','Roll estimate','Color','w');
plot(t, rad2deg(rollAcc),  'Color', [0.6 0.6 0.6]); hold on;
plot(t, rad2deg(rollGyro), 'r', 'LineWidth', 1.2);
plot(t, rad2deg(roll),     'b', 'LineWidth', 1.5);
if useSimulated
    plot(t, rad2deg(rollTrue), 'k--', 'LineWidth', 1.2);
    legend('Accelerometer only','Gyro only','Complementary filter','True','Location','best');
else
    legend('Accelerometer only','Gyro only','Complementary filter','Location','best');
end
grid on; xlabel('Time (s)'); ylabel('Roll (deg)');
title(sprintf('Roll: complementary filter (\\alpha = %.2f)', alpha));

figure('Name','Pitch estimate','Color','w');
plot(t, rad2deg(pitchAcc),  'Color', [0.6 0.6 0.6]); hold on;
plot(t, rad2deg(pitchGyro), 'r', 'LineWidth', 1.2);
plot(t, rad2deg(pitch),     'b', 'LineWidth', 1.5);
if useSimulated
    plot(t, rad2deg(pitchTrue), 'k--', 'LineWidth', 1.2);
end
grid on; xlabel('Time (s)'); ylabel('Pitch (deg)');
title('Pitch: complementary filter');

%% 7) Error and alpha tuning (simulated data only)
if useSimulated
    rmsDeg = @(e) rad2deg(sqrt(mean(e.^2)));
    fprintf('RMS roll error\n');
    fprintf('  Accelerometer only : %.2f deg\n', rmsDeg(rollAcc  - rollTrue));
    fprintf('  Gyro only          : %.2f deg\n', rmsDeg(rollGyro - rollTrue));
    fprintf('  Complementary      : %.2f deg\n', rmsDeg(roll     - rollTrue));

    alphas = [0.80 0.90 0.95 0.98 0.99 0.995 0.999];
    errs   = zeros(size(alphas));
    for j = 1:numel(alphas)
        a = alphas(j);
        r = zeros(N,1); r(1) = rollAcc(1);
        for k = 2:N
            r(k) = a*(r(k-1) + gx(k)*dt(k)) + (1-a)*rollAcc(k);
        end
        errs(j) = rmsDeg(r - rollTrue);
    end

    figure('Name','Alpha tuning','Color','w');
    semilogx(1 - alphas, errs, 'o-', 'LineWidth', 1.5);
    grid on; xlabel('1 - \alpha  (weight on accelerometer)');
    ylabel('RMS roll error (deg)');
    title('Tuning the complementary filter');
end
