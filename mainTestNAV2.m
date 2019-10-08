%%%
%%IMU模拟建模
clc
close all;
clear all;
params = gyroparams
N = 1000;
Fs = 100;
Fc = 0.25;

t = (0:(1/Fs):((N-1)/Fs)).';
acc = zeros(N, 3);
angvel = zeros(N, 3);
angvel(:,1) = sin(2*pi*Fc*t);

imu = imuSensor('SampleRate', Fs, 'Gyroscope', params);
[~, gyroData] = imu(acc, angvel);

figure
plot(t, angvel(:,1), '--', t, gyroData(:,1))
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Ideal Gyroscope Data')
legend('x (ground truth)', 'x (gyroscope)')

if 0
imu = imuSensor('SampleRate', Fs, 'Gyroscope', params);

%%%各种错误
imu.Gyroscope.MeasurementRange = 0.5; % rad/s
imu.Gyroscope.Resolution = 0.5; % (rad/s)/LSB
imu.Gyroscope.AxesMisalignment = [2 0 0]; % percent
imu.Gyroscope.NoiseDensity = 1.25e-2; % (rad/s)/sqrt(Hz)
imu.Gyroscope.BiasInstability = 2.0e-2; % rad/s
imu.Gyroscope.RandomWalk = 9.1e-2; % (rad/s)*sqrt(Hz)
imu.Gyroscope.TemperatureBias = 0.06; % (rad/s)/(degrees C)
imu.Temperature = 35;
imu.Gyroscope.AccelerationBias = 0.3; % (rad/s)/(m/s^2)
%%%

[~, gyroData] = imu(acc, angvel);

figure
plot(t, angvel(:,1), '--', t, gyroData(:,1))
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Saturated Gyroscope Data')
legend('x (ground truth)', 'x (gyroscope)')




end