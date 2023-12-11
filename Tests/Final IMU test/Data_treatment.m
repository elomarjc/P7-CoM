clear all
close all
clc
opts = detectImportOptions('IMU_measurements.csv');

M = readtable('IMU_measurements.csv',opts);
Fs = 50;


Acc = [M.Ax,M.Ay,M.Az]*9.81;
Gyr = [M.Gx,M.Gy,M.Gz]/180*pi;
Mag = [M.Mx,M.My,M.Mz];
Acc = Acc-repmat([0,0,-9.81],size(Acc,1),1);
T = (1:1:max(size(Acc)))/Fs;
T = [T',T',T'];
%% Accel calibration
figure(1)
disp("Accelerometer:");
Acc_mean = mean(Acc)
Acc_noise = Acc-repmat(Acc_mean,size(Acc,1),1);
Acc_power = sum(Acc_noise.*Acc_noise)/max(size(Acc_noise))
plot(T,Acc);
title("Accelerometer data");
legend("X-axis","Y-axis","Z-axis")
grid on;

ylabel("data[N]");
xlabel("Time [s]");
%% Gyroscope calibration
disp("Gyroscope:");

figure(2)
Gyr_mean = mean(Gyr)

Gyr_noise = Gyr-repmat(Gyr_mean,size(Gyr,1),1);
Gyr_power = sum(Gyr_noise.*Gyr_noise)/max(size(Gyr_noise))
plot(T,Gyr);
title("Gyroscope");
legend("X-axis","Y-axis","Z-axis")
grid on;

ylabel("data [rad/s]");
xlabel("Time [s]");
%% Magnetometer calibration
disp("Magnetometer:");
figure(3)

Mag_mean = mean(Mag)

Mag_noise = Mag-repmat(Mag_mean,size(Mag,1),1);
Mag_power = sum(Mag_noise.*Mag_noise)/max(size(Mag_noise))
plot(T,Mag);
title("Magnetometer");
legend("X-axis","Y-axis","Z-axis");
grid on;
ylabel("data [uT]");
xlabel("Time [s]");