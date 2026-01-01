#include <ACAN2515.h>
#include "RMDX.h"
#include <PID_v1.h>

// Configure CAN bus adapter.
static const byte MCP2515_CS  = 10 ; // CS input of MCP2515 (adapt to your design)
static const byte MCP2515_INT =  3 ; // INT output of MCP2515 (adapt to your design)
ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ;

// Select the quartz frequency of your MPC2515 - 8MHz or 16MHz are often used.
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 8 MHz quartz
// Select CAN frequency: 500kbps or 1Mbps can be used.
// Note however that, if a 8MHz quartz is used on the MPC2515, only 500kbps can be used, 1MHz is too fast.
static const uint32_t CAN_BAUDRATE = 500UL * 1000UL; // 500kpbs CAN

RMDX motor(&can); // Create the motor

int fsr_1_AnalogPin = 0; // FSR is connected to analog 0
int fsr_2_AnalogPin = 1; // FSR is connected to analog 0
int fsr_1_Reading;      // the analog reading from the FSR resistor divider
int fsr_2_Reading;      // the analog reading from the FSR resistor divider
int fsr_1_Reading_raw;  
int fsr_2_Reading_raw;  
double Pos_current;
double Neg_current;
double currentPos;
double R_Pos;
double A_Pos;

// Create PID objects
double input, output, setpoint;
PID motorPID(&input, &output, &setpoint, 0.1, 0.0, 0.01, DIRECT); // Example PID constants


void setup () {
    // Use serial port for debug: use serial plotter to view the current / target speed curve.
    Serial.begin (38400) ;
    // Configure MCP2515
    SPI.begin () ;
    ACAN2515Settings settings (QUARTZ_FREQUENCY, CAN_BAUDRATE);
    settings.mRequestedMode = ACAN2515Settings::NormalMode;
    const uint16_t errorCode = can.begin (settings, [] { can.isr () ; });
    // Enable the motor with ID 1 (i.e. 141), and give it some time to start up.
    motor.enable(1);
    delay(1000);

// Initialize PID
  input = 0;
  setpoint = 0;
  output = 0;
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-100, 100); // Adjust limits according to your motor driver
}

void loop () {
  fsr_1_Reading = analogRead(fsr_1_AnalogPin);
  fsr_2_Reading = analogRead(fsr_2_AnalogPin);

  fsr_1_Reading = map(fsr_1_Reading, 850, 950, 0, 100);
  fsr_2_Reading = map(fsr_2_Reading, 850, 950, 0, 100); 

if (fsr_1_Reading < 0)
    fsr_1_Reading = 0;
else if (fsr_1_Reading > 100)
    fsr_1_Reading = 100;

if (fsr_2_Reading < 0)
    fsr_2_Reading = 0;
else if (fsr_2_Reading > 100)
    fsr_2_Reading = 100;


  input = -(fsr_1_Reading - fsr_2_Reading);

    // Compute PID control
  motorPID.Compute();

    // Apply the control output to the motor
  if (output > 0) {
    //analogWrite(MOTOR_PIN, output);
   // Serial.print(output);
  } else {
    //analogWrite(MOTOR_PIN, -output);
    //Serial.print(-output);
  }

  // Print debugging information
  Serial.print("FSR1: ");
  Serial.print(fsr_1_Reading);
  Serial.print(" - FSR2: ");
  Serial.print(fsr_2_Reading);
  Serial.print(" - Input: ");
  Serial.print(input);
  Serial.print(" - Output: ");
  Serial.println(output*10);

  delay(100); // Adjust the delay as needed

} // end of the loop