% Read data from the Excel file
data = importdata("stepResponse.csv")

% Extract time and dB values
time = data(:, 1); % Assuming the time data is in the first column
position = data(:, 2);   % Assuming the dB data is in the second column

% Set the target y-value
tau = 5.12*0.63;

% Perform linear regression up to the point where y = 3.36
regressionRange = time <= 0.11017;
coeff = polyfit(time(regressionRange), position(regressionRange), 1)

% Ramp values (remember to use SI units):
m = 0.32; % Slope of steady-state ramp
n = -0.19;   % Y-axis intercept 
V_in = 10*127/255; % Applied input size

% Calculations ------------------------------------------------------
syms s A B V
disp("The ramp response should follow the formula:");
G = 1/(s^2*(A*s+B));
alpha = ilaplace(G*V);

V = V_in;
B = V/coeff(1);
A = (-coeff(2))*B^2/V;

disp("And the TF is:");
s  = tf('s');
G = (1/B)/(A/B*s+1)/s

% Create the plot
figure;
plot(time, position, 'b-','LineWidth', 2, 'DisplayName', 'Test');
xlabel('Time[s]');
ylabel('Position[cm]');
title('Step Response of potentiometer slides DC-motor');
grid on;

% Set the x-axis limit to show values up to 0.3
xlim([0, 0.25]);

% Calculate and plot the step response of G
t = 0:0.01:0.3; % Define the time vector for the step response
[y, t] = step(G*V, t);

hold on;
plot(t, y, 'g-', 'LineWidth', 2, 'DisplayName', 'tf function');
hold off;

legend('Test', 'Transfer function');