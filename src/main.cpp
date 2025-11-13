

// void setup() {
//   Serial.begin(9600);  // USB serial
//   mySerial.begin(420000); // UART0
// }

// void loop() {
  
//   if (mySerial.available()) {
//     char data = mySerial.read();
//     // Serial.print("Received on UART0: ");
//     Serial.print(data);
//   }
// }
#include <HardwareSerial.h>

#include <Arduino.h>
#include "crsf.h"

#define A_IN1 1  // Motor A INA
#define A_IN2 2  // Motor A INB

// Speed of the motor (PWM)
int motorSpeed = 128; // Range: 0 to 255


#define RXD2 17
#define TXD2 16
#define SBUS_BUFFER_SIZE 25
uint8_t _rcs_buf[25] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};

int aileronsPin = 12;
int elevatorPin = 13;
int throttlePin = 14;
int rudderPin = 15;

int aileronsPWMChannel = 1;
int elevatorPWMChannel = 2;
int throttlePWMChannel = 3;
int rudderPWMChannel = 4;
HardwareSerial mySerial(0); // UART0 (Serial0)

void SetServoPos(float percent, int pwmChannel)
{
    // 50 cycles per second 1,000ms / 50 = 100 /5 = 20ms per cycle
    // 1ms / 20ms = 1/20 duty cycle
    // 2ms / 20ms = 2/20 = 1/10 duty cycle
    // using 16 bit resolution for PWM signal convert to range of 0-65536 (0-100% duty/on time)
    // 1/20th of 65536 = 3276.8
    // 1/10th of 65536 = 6553.6

    uint32_t duty = map(percent, 0, 100, 3276.8, 6553.6);

   // ledcWrite(pwmChannel, duty);
}

void runMotor1(int speed) {

  Serial.print("speed: ");
   Serial.print(speed);
   Serial.print(" c:  ");
  if (speed > 55) {
    // Move forward
    analogWrite(A_IN1, map(speed,50,110,0,255));
    Serial.print(map(speed,50,110,0,255));
    analogWrite(A_IN2, 0);
  } else if (speed < 45) {
    // Move backward
    analogWrite(A_IN1, 0);
    analogWrite(A_IN2, map(speed,-5,50,255,0));
    Serial.print(map(speed,-5,50,255,0));
  } else {
    // Stop
    analogWrite(A_IN1, 0);
    analogWrite(A_IN2, 0);
  }
  // Set motor speed (PWM)
  // analogWrite(ENA, speed); // Assuming ENA is the PWM pin for Motor A
  Serial.println();
}


void setup() {

  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);

  mySerial.begin(420000); // UART0
  Serial.begin(460800);  // USB serial
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  //Serial.begin(460800);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  // Serial0.begin(420000, SERIAL_8N1, RXD2, TXD2);
  // Serial.println("Serial Txd is on pin: "+String(TX));
  // Serial.println("Serial Rxd is on pin: "+String(RX));
  
  // ledcSetup(aileronsPWMChannel,50,16);
  // ledcSetup(elevatorPWMChannel,50,16);
  // ledcSetup(throttlePWMChannel,50,16);
  // ledcSetup(rudderPWMChannel,50,16);

  // ledcAttachPin(aileronsPin, aileronsPWMChannel);
  // ledcAttachPin(elevatorPin, elevatorPWMChannel);
  // ledcAttachPin(throttlePin, throttlePWMChannel);
  // ledcAttachPin(rudderPin, rudderPWMChannel);
}
int counter = 0;
void loop() { //Choose Serial1 or Serial2 as required
 
  while (mySerial.available()) {
    //Serial.print("c");
    size_t numBytesRead = mySerial.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
    if(numBytesRead > 0)
    {
      crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS );
      // Serial.print("Channel 1: ");
      // Serial.print(_raw_rc_values[0]);
      // Serial.print("\tChannel 2: ");
      // Serial.print(_raw_rc_values[1]);
      // Serial.print("\tChannel 3: ");
      // Serial.print(_raw_rc_values[2]);
      // Serial.print("\tChannel 4: ");
      // Serial.print(_raw_rc_values[3]);
      // Serial.print("\tChannel 5: ");
      // Serial.println(_raw_rc_values[4]);

      int aileronsMapped = map(_raw_rc_values[0], 1000, 2000, 0, 100);
      int elevatorMapped = map(_raw_rc_values[1], 1000, 2000, 0, 100);
      int throttleMapped = map(_raw_rc_values[2], 1000, 2000, 0, 100);
      int rudderMapped = map(_raw_rc_values[3], 1000, 2000, 0, 100);
      int switchMapped = map(_raw_rc_values[4], 1000, 2000, 0, 100);

      SetServoPos(aileronsMapped, aileronsPWMChannel);
      SetServoPos(elevatorMapped, elevatorPWMChannel);
      SetServoPos(throttleMapped, throttlePWMChannel);
      SetServoPos(rudderMapped, rudderPWMChannel);

                   // Move forward for 2 seconds

      runMotor1(throttleMapped);
    }
  }
  
}