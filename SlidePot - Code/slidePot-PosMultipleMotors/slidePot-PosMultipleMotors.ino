// Pins for the motor driver module
int enA = 4; // Enable for Motor A
int in1 = 2; // Input 1 for Motor A
int in2 = 15; // Input 2 for Motor A
double potA = 35; // Potentiometer for Motor A

int enB = 5; // Enable for Motor B
int in3 = 4; // Input 1 for Motor B
int in4 = 3; // Input 2 for Motor B
double potB = 35; // Potentiometer for Motor A

//int enC = 6; // Enable for Motor C
//int in5 = 11; // Input 1 for Motor C
//int in6 = 10; // Input 2 for Motor C
//double potC = 35; // Potentiometer for Motor C

// try lowering freq, to vibrate, diddering signal
// Get rid of deadband

// Setting PWM properties
const int freq = 300;   //300 hz is perfect
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 0; // Initialize dutyCycle to 0

// Define the range of potentiometer values for leftmost and rightmost positions
const double minPotValue = 4095.0; // Minimum potentiometer value (leftmost)
const double maxPotValue = 0.0;    // Maximum potentiometer value (rightmost)

// Define the range of positions in centimeters for leftmost and rightmost positions
const double minPositionLimit = -4.0; // Minimum position in cm
const double maxPositionLimit = 4.0;  // Maximum position in cm

// Target position
double targetPositionA = 0;
double targetPositionB = 0;
//double targetPositionC = 0;

// PID Controller Parameters
double kp = 1.5; // Proportional gain
double ki = 0; // Integral gain
double kd = 0; // Derivative gain
double integral = 0.0;
double previousError = 0.0;

// Function to calculate the voltage from the position (cm)
double calculatePositionToVoltage(double position) {
  return 0.0052 * position * position * position - 0.0288 * position * position + 0.1114 * position - 0.0161;
}

double calculateVoltageToPosition(double voltage) {
  // Calculate the position within the original range
  return 0.8041 * voltage * voltage * voltage - 5.0581 * voltage * voltage + 10.821 * voltage + 0.5937 - 5;
}

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(potA, INPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(potB, INPUT);
  
//  pinMode(enC, OUTPUT);
//  pinMode(in5, OUTPUT);
//  pinMode(in6, OUTPUT);
//  pinMode(potC, INPUT);

  ledcSetup(pwmChannel, freq, resolution);  // Configure LED PWM functionality
  
  ledcAttachPin(enA, pwmChannel); // Attach the channel to the GPIO to be controlled
  ledcAttachPin(enB, pwmChannel);
  //ledcAttachPin(enC, pwmChannel);

  Serial.begin(115200);
  Serial.print("Testing DC Motor...");
}

void loop() {
  SlideToPosition(targetPositionA, enA, in1, in2, potA);
  SlideToPosition(targetPositionB, enB, in3, in4, potB);
  //SlideToPosition(targetPositionA, enA, in1, in2, potA);

  delay(100);
}

void SlideToPosition(double targetPosition, int enX, int inX1, int inX2, double potX) {
  double potValue = analogRead(potX);
  double potVoltage = analogRead(potX) * (3.3 / minPotValue);
  double currentPosition = calculateVoltageToPosition(potVoltage);

  // Check and enforce position limits
  if (targetPosition < minPositionLimit) {
    targetPosition = minPositionLimit;
  } else if (targetPosition > maxPositionLimit) {
    targetPosition = maxPositionLimit;
  }

  Serial.print("Pot raw: ");
  Serial.print(potValue);
  Serial.print(" CurrentPosition: ");
  Serial.print(currentPosition);
  Serial.print(" TargetPosition: ");
  Serial.print(targetPosition);
  
  // Calculate the error (difference between current and target position)
  double error = targetPosition - currentPosition;
  error = -error;
  Serial.print(" Error: ");
  Serial.println(error);

  // Calculate PID terms
  double proportionalTerm = kp * error;
  integral += ki * error;
  double derivativeTerm = kd * (error - previousError);

  // Calculate the control signal (duty cycle)
  double controlSignal = proportionalTerm + integral + derivativeTerm;

  // Update the previous error
  previousError = error;

  // Limiting signal
  double abs_signal = abs(controlSignal);
  double maxControlSignal = 10.0;

  if (abs_signal >= maxControlSignal) {
    controlSignal = controlSignal / abs_signal * maxControlSignal;
  }

  // Determine the direction of rotation
  if (controlSignal > 0) {
    digitalWrite(inX1, LOW);
    digitalWrite(inX2, HIGH);
  } else if (controlSignal < 0) {
    digitalWrite(inX1, HIGH);
    digitalWrite(inX2, LOW);
  } else {
    // Stop the motor if the target position is reached
    ledcWrite(pwmChannel, LOW);
  }

  // Apply static friction compensation
  if (controlSignal > 0) {  // going left
    double staticFrictionCompensation = 1.2;
    controlSignal += staticFrictionCompensation;

  } else if (controlSignal < 0) {  // going right
    double staticFrictionCompensation = 1.2;
    controlSignal -= staticFrictionCompensation;
  }

  // mapdouble the control signal to the duty cycle range (0-255)
  dutyCycle = mapdouble(abs(controlSignal), 0, maxControlSignal, 0, 255);
  ledcWrite(pwmChannel, dutyCycle);
}

double mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
