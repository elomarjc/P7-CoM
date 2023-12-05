clc
% Read data from the Excel file
data1 = importdata("magnetometer_with_calibration.txt");
data2 = importdata("magnetometer_without_calibration.txt");

% Normilizing the calibrated measurements
%for i=1:size(data1,1)
%    n_vec = norm(data1(i, 1:3));
%    Calibrated_magnX(i) = data1(i, 1) / n_vec;
%    Calibrated_magnY(i) = data1(i, 2) / n_vec;  
%    Calibrated_magnZ(i) = data1(i, 3) / n_vec;
%end
% calibrated measurements
Calibrated_magnX = data1(:, 1);
Calibrated_magnY = data1(:, 2);
Calibrated_magnZ = data1(:, 3);
% uncalibrated measurements
magnX = data2(:, 1);
MaxX = max(magnX);
MinX = min(magnX);
offsetX = (MaxX + MinX) / 2
deltaX = (MaxX - MinX) / 2


magnY = data2(:, 2);  
MaxY = max(magnY);
MinY = min(magnY);
offsetY = (MaxY + MinY) / 2
deltaY = (MaxY - MinY) / 2

magnZ = data2(:, 3);
MaxZ = max(magnZ);
MinZ = min(magnZ);
offsetZ = (MaxZ + MinZ) / 2
deltaZ = (MaxZ - MinZ) / 2

avgDelta = (deltaX + deltaY + deltaZ) / 3
scaleX = avgDelta / deltaX
scaleY = avgDelta / deltaY
scaleZ = avgDelta / deltaZ

% Calibration done in matlab
matCali_X = (magnX - offsetX) * scaleX;
matCali_Y = (magnY - offsetY) * scaleY;
matCali_Z = (magnZ - offsetZ) * scaleZ;

% Plot (X, Y), (Y, Z), and (Z, X) on the same graph
figure;
scatter(Calibrated_magnX, Calibrated_magnY, 'o', 'DisplayName', '(X, Y)');
hold on;
scatter(Calibrated_magnY, Calibrated_magnZ, 'x', 'DisplayName', '(Y, Z)');
scatter(Calibrated_magnZ, Calibrated_magnX, 's', 'DisplayName', '(Z, X)');

title('Magnetometer with calibration');
xlabel('Magnetic field strength [\mu T]');
ylabel('Magnetic field strength [\mu T]');
legend('Location', 'Best');
grid on;
hold off;

% Scatter plot for (X, Y), (Y, Z), and (Z, X) on the same graph
figure;
scatter(magnX, magnY, 'o', 'DisplayName', '(X, Y)');
hold on;
scatter(magnY, magnZ, 'x', 'DisplayName', '(Y, Z)');
scatter(magnZ, magnX, 's', 'DisplayName', '(Z, X)');

title('Magnetometer without calibration and normalization');
xlabel('Magnetic field strength [\mu T]');
ylabel('Magnetic field strength [\mu T]');
legend('Location', 'Best');
grid on;
hold off;

% Scatter plot for (X, Y), (Y, Z), and (Z, X) on the same graph
figure;
scatter(matCali_X, matCali_Y, 'o', 'DisplayName', '(X, Y)');
hold on;
scatter(matCali_Y, matCali_Z, 'x', 'DisplayName', '(Y, Z)');
scatter(matCali_Z, matCali_X, 's', 'DisplayName', '(Z, X)');

title('Magnetometer calibration done in matlab');
xlabel('Magnetic field strength [\mu T]');
ylabel('Magnetic field strength [\mu T]');
legend('Location', 'Best');
grid on;
hold off;

% Scatter plot in 3D
%figure;
%scatter3(Calibrated_magnX, Calibrated_magnY, Calibrated_magnZ, 'filled', 'DisplayName', 'Data Points');
%title('3D Scatter Plot');
%xlabel('X');
%ylabel('Y');
%zlabel('Z');
%legend('Location', 'Best');
%grid on;
%axis equal; % Make axis scales equal for better visualization