% Read data from the Excel file
data = importdata("Controller_step_response_test.txt");

% Extract time and dB values
time = data(:, 1)/1000;
position = data(:, 2);  
voltage = data(:, 3);
position = position -position(1);

timeSim = out.response.Time;
positionSim = out.response.Data;  
positionSim = positionSim -positionSim(1);

% Create the plot
figure;
plot(time, position, 'b-','LineWidth', 2, 'DisplayName', 'Test');
xlabel('Time[s]');
ylabel('Position[cm]');
title('Step Response with controller');
grid on;

% Set the x-axis limit to show values up to 0.75
xlim([0, 0.3]);
hold on;
plot(timeSim, positionSim, 'g-','LineWidth', 2, 'DisplayName', 'Simulated');
hold off;
