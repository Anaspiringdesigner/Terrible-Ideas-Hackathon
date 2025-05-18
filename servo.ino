#include <Servo.h>
#define dirPin 2
#define stepPin 3
#define stepsPerRevolution 200 // Adjust this value based on your stepper motor's specification

int motorSpeed = 100; // Fixed motor speed
Servo pumpm1; // create servo object to control pump motor 1
Servo pumpm2; // create servo object to control pump motor 2
const int pumpm1pin = A1; // analog pin used to connect the potentiometer
const int pot = A0;
int val; // variable to read the value from the analog pin
int val1;
const int leftButtonPin = 13; // Pin for left buttonconst 
int rightButtonPin = 12; // Pin for right buttonconst 
int swtch = 11;float currentAngle = 0; // Track the current position of the stepper motor in degrees
float x = 0.0;
float y = 0.0;
String receivedData = ""; // String to hold incoming data


void setup() {
    // Declare pins as output:
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    // Setup button pins as input with pull-up resistors  
    pinMode(leftButtonPin, INPUT_PULLUP);
    pinMode(rightButtonPin, INPUT_PULLUP);
    pinMode(swtch, INPUT);
    
    pumpm1.attach(9); // attaches the servo on pin 9  
    pumpm2.attach(10); // attaches the servo on pin 10

    Serial.begin(9600);

}

/ Function to step the motor
void stepMotor(int speed) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(speed);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(speed);
}

// Function to move the stepper motor to a specific angle
void moveStepperToAngle(float targetAngle) {
    // Calculate the number of steps required to move to the target angle
    float degreesPerStep = 360.0 / stepsPerRevolution;
    int stepsNeeded = (targetAngle - currentAngle) / degreesPerStep;
  
    // Set the direction of rotation
    if (stepsNeeded > 0) {
      digitalWrite(dirPin, HIGH); // Clockwise
    } else {
      digitalWrite(dirPin, LOW); // Counterclockwise
      stepsNeeded = -stepsNeeded; // Make stepsNeeded positive
    }
  
     // Step the motor the required number of steps
  for (int i = 0; i < stepsNeeded; i++) {
    stepMotor(motorSpeed); // Step the motor
  }
  // Update the current angle
  currentAngle = targetAngle;
}

void parseData(String data) {
    int commaIndex = data.indexOf(',');
    
    if (commaIndex > 0) {
      String xStr = data.substring(0, commaIndex); // Extract X value
      String yStr = data.substring(commaIndex + 1); // Extract Y value
      
      // Convert strings to floats
      x = xStr.toFloat();
      y = yStr.toFloat();
      
      // Debugging print to check values
      Serial.print("x: ");
      Serial.print(x);
      Serial.print(" | y: ");
      Serial.println(y);
    }
  }

  void loop() {
     if (digitalRead(swtch) == HIGH){

    // Read the state of the buttons    
    int leftButtonState = digitalRead(leftButtonPin);
    int rightButtonState = digitalRead(rightButtonPin);

    // Read potentiometer value to adjust motor speed    
    int potValue = analogRead(pot);
    motorSpeed = map(potValue, 0, 1023, 2000, 10000); // Map to a usable range (slower = larger delay)

     // Stepper motor control based on button presses
     if (leftButtonState == LOW && rightButtonState == HIGH) { // Left button pressed
        digitalWrite(dirPin, HIGH); // Rotate clockwise
        stepMotor(motorSpeed);
      } 
      else if (rightButtonState == LOW && leftButtonState == HIGH) { // Right button pressed
        digitalWrite(dirPin, LOW); // Rotate counterclockwise
        stepMotor(motorSpeed);
      } 
      else { // Neither button pressed
        digitalWrite(stepPin, LOW); // Stop motor when no button is pressed
      }


    // Example of moving the stepper motor to a certain angle
    // moveStepperToAngle(90); // Uncomment to move to 90 degrees

     // Control servos (unchanged)
     val = analogRead(pumpm1pin); // reads the value of the potentiometer (value between 0 and 1023)
     val = map(val, 0, 1023, 100, 200); // scale it to use it with the servo (value between 100 and 200)
     pumpm1.write(val); // sets the servo position according to the scaled value
     Serial.print(pumpm1.read());
     Serial.println(" : pumpm1");
     val1 = analogRead(pumpm1pin); // reads the value of the potentiometer (value between 0 and 1023)
     val1 = map(val1, -200, 1023, 100, 0); // scale it to use it with servo (Value between 100 and 0)
     pumpm2.write(val1);
     Serial.print(pumpm2.read());
     Serial.println(" : pumpm2");

    }else{

        if (Serial.available() > 0) {
          // Read the incoming byte:
          char incomingByte = Serial.read();
          
          // If we receive a newline character, we process the data
          if (incomingByte == '\n') {
            // Split the received data into two angles
            parseData(receivedData);
            receivedData = "";
          }
          else {
            // Append incoming characters to the receivedData string
            receivedData += incomingByte;
          }
        }
    
    int targetAngle = map(x, -45, 45, 90, -90);
    moveStepperToAngle(targetAngle);
    int rotate1 = map(y, -45, 45, 170, 90);
    pumpm1.write(rotate1);
    int rotate2 = map(y, -45, 45, 14, 94);
    pumpm2.write(rotate2);
  }

}