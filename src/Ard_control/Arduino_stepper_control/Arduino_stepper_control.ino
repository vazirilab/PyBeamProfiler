// In this skitch you can change the pin numbers for PUL and DIR to suit your Arduino.
// Stepper 1 is for the x direction and 2 is for the y direction

const int ENA1 = 4;
const int OPTO1 = 5;
const int ENA2 = 10;
const int OPTO2 = 11;
const int PUL1 = 2;
const int DIR1 = 3;
const int PUL2 = 8;
const int DIR2= 9;
int  direction,steps;
char motor;
//char motor = 'X';
//int direction = 0;
//int steps = 0;
 //AccelStepper stepper1(AccelStepper::DRIVER, 2, 3); // 2 is for PUL and 3 is for DIR
 //AccelStepper stepper2(AccelStepper::DRIVER, 8, 9); 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ENA1, OUTPUT);
  pinMode(OPTO1, OUTPUT);
  pinMode(PUL1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PUL2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(OPTO2, OUTPUT);
  digitalWrite(OPTO1, HIGH);
  digitalWrite(OPTO2, HIGH);
  digitalWrite(ENA1, HIGH);
  digitalWrite(ENA2, HIGH);
  delay(1);
//  stepper1.setMaxSpeed(3000000);
  //stepper2.setMaxSpeed(3000000);
//  stepper2.setSpeed(2800);
//  stepper1.setSpeed(2800);
//  stepper1.setAcceleration(1000000.0);
 // stepper2.setAcceleration(1000000.0);
//  stepper1.setMinPulseWidth(10);
  //stepper2.setMinPulseWidth(10);
  
  digitalWrite(13, LOW);
}

void loop() {
  
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read the command from Python
    
    
    sscanf(command.c_str(), "%c,%d,%d", &motor, &direction, &steps);
    
    if (motor == 'X') {
      
 //     if (stepper1.isRunning()) {
   //     stepper1.stop();
     //   stepper1.runToPosition();
        
   //   }
      
      if (direction == 0) {
        digitalWrite(3, LOW);
        delayMicroseconds(50);
        //int s = (steps.toInt());
        for (int i = 1; i <= steps; i++) {
           delayMicroseconds(8);
           digitalWrite(2, HIGH);
           delayMicroseconds(8);
           digitalWrite(2, LOW);
           
      }
        

      }else if (direction == 1) {
        //int s = (steps);
        
        digitalWrite(3, HIGH);
        delayMicroseconds(50);
        //int s = (steps.toInt());
        for (int i = 1; i <= steps; i++) {
           delayMicroseconds(8);
           digitalWrite(2, HIGH);
           delayMicroseconds(8);
           digitalWrite(2, LOW);
           
        }
        
      }
      
      
 //     for (int i = 0; i <= 255; i++) {
 //          delayMicroseconds(1000);
 //          digitalWrite(2, HIGH);
 //          delayMicroseconds(1000);
 //          digitalWrite(2, LOW);
  //    }
 //     for (int i = 0; i <= 255; i++) {
 //          delayMicroseconds(1000);
 //          digitalWrite(8, HIGH);
 //          delayMicroseconds(1000);
 //          digitalWrite(8, LOW);
 //     }

  }else if (motor == 'Y') {
 //     if (stepper2.isRunning()) {
   //     stepper2.stop();
     //   stepper2.runToPosition();
        
   //   }
      if (direction == 0) {
        digitalWrite(9, LOW);
        delayMicroseconds(50);
        for (int i = 1; i <= steps; i++) {
           delayMicroseconds(8);
           digitalWrite(8, HIGH);
           delayMicroseconds(8);
           digitalWrite(8, LOW);
           
        }
        
      }else if (direction == 1) {
        digitalWrite(9, HIGH);
        delayMicroseconds(50);
        for (int i = 1; i <= steps; i++) {
           delayMicroseconds(8);
           digitalWrite(8, HIGH);
           delayMicroseconds(8);
           digitalWrite(8, LOW);
           
        }

      }
      
      
    }
  }else if (motor == 'B') {
    for (int i = 0; i < 20; i++){
      delay(0.5);
      digitalWrite(13, HIGH);
      delay(0.5);
      digitalWrite(13, LOW);
    }
  }
 // stepper1.runSpeedToPosition();
  //stepper2.runSpeedToPosition();

  // put your main code here, to run repeatedly:

}
