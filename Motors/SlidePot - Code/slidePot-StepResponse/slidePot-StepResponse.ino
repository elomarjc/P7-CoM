#define F 200

// Pins
int potPin = 35;
int in1 = 2;
int in2 = 15;
int enA = 4;

// Setting PWM properties
const int freq = 30000;   
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 0; // Initialize dutyCycle to 0

// Define the range of potentiometer values for leftmost and rightmost positions
const int minPotValue = 4095; // Minimum potentiometer value (leftmost)
const int maxPotValue = 0;    // Maximum potentiometer value (rightmost)

// Define the range of positions in centimeters for leftmost and rightmost positions
const int minPositionLimit = -4; // Minimum position in cm
const int maxPositionLimit = 4;  // Maximum position in cm

// Target position
double targetPositionNow = 0;

// Max voltage
double maxMotorVoltage = 10.0;

// PID Controller Parameters
double kp = 1.5; // Proportional gain
double ki = 0; // Integral gain
double kd = 0; // Derivative gain
double integral = 0.0;
double previousError = 0.0;

// Time variables
unsigned long startTime = 0;
unsigned long currentTime = 0;

// Function to calculate the voltage from the position (cm)
double calculatePositionToVoltage(double position) {
  return 0.0052 * position * position * position - 0.0288 * position * position + 0.1114 * position - 0.0161;
}

double calculateVoltageToPosition(double voltage) {
  // Calculate the position within the original range
  return 0.8041 * voltage * voltage * voltage - 5.0581 * voltage * voltage + 10.821 * voltage + 0.5937 - 5;
}

void setup() {
  pinMode(potPin, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Configure LED PWM functionality
  ledcSetup(pwmChannel, freq, resolution);

  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(enA, pwmChannel);

  Serial.begin(115200);
  while(!Serial);

  delay(1000); // Wait for 1 second before starting the step response

  // Record the start time
  startTime = micros();
}

void loop() {

  // Calculate the elapsed time
  currentTime = micros();
  unsigned long  elapsedTime = (currentTime - startTime); // Convert to seconds

  // Step response test: Change the target position after a certain time
  if (elapsedTime < 2 * 1000000) { // For the first 2 seconds, maintain the initial target
    targetPositionNow = 0;
    SlideToPosition(targetPositionNow);
  } else {
    Serial.print(elapsedTime/1000.0-2000); // Time in seconds
    Serial.print('\t');
    Serial.println(getPotentiometer());
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW); 
    setVoltageMotor(5);  // 5 volt
  }
  //Serial.print("Elapsed Time: ");
  while (micros() - currentTime < 1000000 / F);

  if (elapsedTime > 4*1000000) {  // After 4 seconds, turn off motor
    ledcWrite(pwmChannel, LOW);   
    while (true);
  }
}

void SlideToPosition(double targetPosition) {

  double currentPosition = getPotentiometer();

  // Check and enforce position limits
  if (targetPosition < minPositionLimit) {
    targetPosition = minPositionLimit;
  } else if (targetPosition > maxPositionLimit) {
    targetPosition = maxPositionLimit;
  }

  //  Serial.print("Pot raw: ");
  //  Serial.print(potValue);
  //  Serial.print(" CurrentPosition: ");
  //  Serial.print(currentPosition);
  //  Serial.print(" TargetPosition: ");
  //  Serial.print(targetPosition);

  // Calculate the error (difference between current and target position)
  double error = targetPosition - currentPosition;
  error = -error;

  //Serial.print('\t');
  //Serial.print(" Error: ");
  //Serial.println(error); // Error

  // Calculate PID terms
  double proportionalTerm = kp * error;
  integral += ki * error;
  double derivativeTerm = kd * (error - previousError);

  // Calculate the control signal (duty cycle)
  double motorVoltage = proportionalTerm + integral + derivativeTerm;

  // Update the previous error
  previousError = error;

  // Limiting signal
  double abs_motorVoltage = abs(motorVoltage);

  if (abs_motorVoltage >= maxMotorVoltage) {
    motorVoltage = motorVoltage / abs_motorVoltage * maxMotorVoltage;
  }

  // Determine the direction of rotation
  if (motorVoltage > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (motorVoltage < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    // Stop the motor if the target position is reached
    ledcWrite(pwmChannel, LOW);
  }

  setVoltageMotor(motorVoltage);
}

double getPotentiometer() {
  double potValue = analogRead(potPin);
  double potVoltage = analogRead(potPin) * (3.3 / minPotValue);
  return calculateVoltageToPosition(potVoltage);
}

void setVoltageMotor(double motorVoltage) {
  
  // Apply static friction compensation (OLD)
//  if (motorVoltage > 0) {  // going left
//    double staticFrictionCompensation = 1.2;
//    motorVoltage += staticFrictionCompensation;
//
//  } else if (motorVoltage < 0) {  // going right
//    double staticFrictionCompensation = 1.2;
//    motorVoltage -= staticFrictionCompensation;
//  }

  // mapdouble the control signal to the duty cycle range (0-255)
  dutyCycle = mapdouble(abs(motorVoltage), 0, maxMotorVoltage, 0, 255);
  ledcWrite(pwmChannel, dutyCycle);
  }

double mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
