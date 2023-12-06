#include "Pot_Control.h"

void POT::Initialize(int potPin_Int, int enA_Int, int in1_Int, int in2_Int, int PWMpin_Int){
    // Initialize the pins
    this->potPin = potPin_Int;
    this->enA = enA_Int;
    this->in1 = in1_Int;
    this->in2 = in2_Int;
    this->PWMpin = PWMpin_Int;
    // Motor setup
    pinMode(potPin, INPUT);
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    // Configure LED PWM functionality
    ledcSetup(PWMpin, 30000, resolution);
    // Attach the channel to the GPIO
    ledcAttachPin(enA, PWMpin);
}

void POT::setVoltageMotor(float motorVoltage, float dz_correction) {
  if (motorVoltage > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (motorVoltage < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    // Stop the motor if the target position is reached
    ledcWrite(PWMpin, LOW);
  }
  // Map motorVoltage to the duty cycle range [0, 255]. Contrain limits motorVoltage from 0 to 10
  int dutyCycle = map(constrain(abs(motorVoltage) + dz_correction, 0, 10), 0, 10, 0, 255);
  ledcWrite(PWMpin, dutyCycle);
}

// Initialize the function values
void POT::InitPotFunction(float fourth_Int, float thrid_Int, float second_Int, float first_Int){
    this->first = first_Int;
    this->second = second_Int;
    this->third = thrid_Int;
    this->fourth = fourth_Int;
}
// Disable deadzone when close to goal
float POT::DeadzoneDisable(float deadzone, float targetPosition){
  if (abs(targetPosition - VoltageToPosition(PotVoltage())) <= 0.03) {
    dz_correction = 0;
  }else {
    dz_correction = deadzone;
  }
  return dz_correction;
}
//Translate voltage to a position on the potentiometer
float POT::VoltageToPosition(float voltage){
    return fourth * voltage * voltage * voltage - third * voltage * voltage + second * voltage + first - 5;
}

//Read the potentiometer voltage value
float POT::PotVoltage(){
    float potVoltage = analogRead(potPin) * (3.3 / minPotValue);
    return potVoltage;
}

void PIController::Initialize() {
    // Clear controller
    integrator = 0.0;
    prevError = 0.0;
    out = 0.0;
}

//Update PI controller output value
float PIController::Update(float setpoint, float measurement) {
    float error = setpoint - measurement;

    float proportional = Kp * error;
    
    // Using the Trapezoidal rule for integration
    integrator = integrator + 0.5 * Ki * T * (error + prevError);

    // Anti-wind-up via dynamic integrator clamping
    if (integrator > limMaxInt) {
      integrator = limMaxInt;
    } else if (integrator < limMinInt) {
      integrator = limMinInt;
    }

    // Compute output and apply limits
    out = proportional + integrator;

    if (out > limMax) {
      out = limMax;
    } else if (out < limMin) {
      out = limMin;
    }

    // Store error for later use
    prevError = error;

    // Return controller output
    return out;
}


