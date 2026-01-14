
// =======================================================================================
//                          Maxstang's 232 Transition Code
// =======================================================================================
//                          Last Revised Date: 6/18/25
//                   Written by: Maxstang, with MAJOR help from Linor and Croy9000
//
// =======================================================================================
//
//    This code was created to drive the widely avaliable BTS-7960 motor driver for use in
//    Droid transitions from two to three legs and back using brushed encoderless motors. It
//    is written to be used on an Arduino Mega.
//
//    Originally created to be used in conjunction with a keyfob remote and SHADOW code running on a separate
//    Arduino Mega, this code is now being controlled by a custom sketch on an ESP32.
//
//    11/6/18   -   added triggered boolean values to reduce missing limit switch activations
//    11/16/18  -   added Wiggle code and code to receive commands from the SHADOW Mega ADK to trigger it
//    12/10/18  -   added Peek code and code to receive commands from the SHADOW Mega ADK to trigger it
//    2/16/19   -   added Look Down code to the Peek command. Droid now looks down when peek command is called in two leg mode.
//    2/28/19   -   added ankle position reset to wiggle code to keep rear wheels off the ground after wiggle.
//    5/31/19   -   adjusted Look Down timing to better center the body
//    5/31/19   -   added Nod code and code to receive commands from the SHADOW Mega ADK to trigger it
//    2/1/21    -   split slow 2-3 center leg deployment into two segments
//    6/9/21    -   added a tiny bit of shoulder rotation before slow 2-3 center leg deployment to fix stalling
//    12/14/24  -   begin modifying code for Dee Dee
//    1/17/25   -   Added new Laugh animation specific to Dee Dee
//    1/21/25   -   Added new Bow/Kneel animation
//    5/29/25   -   Abandoned porting, now using ESP32 as control interface.  Adjusted code for new animations and communication
//    6/2/25    -   Returned to HIGH activation due to false LOW triggering during brown outs.
//    6/18/25   -   Reversed motor directions to reflect 232 PCB construction
//    6/18/25   -   Revised limit switch pins to reflect 232 PCB construction
//
//
//
// 232 Motor Identification and pin assignment
//
// Shoulder BTS-7960
int RPWMS = 45;  //PWM signal for "right" rotation of the shoulder motor 44
int LPWMS = 44;  //PWM signal for "left" rotation of the shoulder motor 45
int R_ENS = 16;  //Enable pin for "right" rotation of the shoulder motor
int L_ENS = 17;  //Enable pin ofor the "left" rotation of the shoulder motor

// Center Leg BTS-7960
int RPWMC = 2;
int LPWMC = 3;
int R_ENC = 4;
int L_ENC = 5;

// Left Ankle BTS-7960
int RPWML = 10;
int LPWML = 11;
int R_ENL = 12;
int L_ENL = 13;

// Right Ankle BTS-7960
int RPWMR = 6;
int LPWMR = 7;
int R_ENR = 8;
int L_ENR = 9;

// Activation Inputs

// ESP32 Controlled

const int THREELEG = 18;       //Transition to Three Leg Mode
const int PEEKESP = 19;           //When in Three Leg Mode, straighten ankles to lean forward
const int TWOLEG = 20;         //Transition to Two Leg Mode
const int CENTERLEGTEST = 21;  //Deploy/retract center leg for testing
const int CHUCKLEESP = 22;        //Retract center leg incrementally, pause, then fully deploy again
const int WIGGLEESP = 23;         //When in Three Leg Mode, alternate straightening and bending ankles
const int NODESP = 32;            //Slighly retract, then extend center ankle repeatedly (todo, add two leg version)
const int KNEELESP = 33;          //When in Three Leg Mode, retract center ankle to kneel/bow.  Press again to slowly rise
const int SADESP = 34;            //When in Three Leg Mode, retract center ankle slowly
const int STANDUPESP = 43;        //When in Three leg mode, deploy center ankle fully (for recovering from SAD or other commands that result in retracted center ankle

// Shadow Controlled

const int REARWHEELLIFT = 46;
const int PEEK = 47;
const int CHUCKLE = 48;
const int WIGGLE = 49;
const int NOD = 50;
const int KNEEL = 51;
const int SAD = 52;
const int STANDUP = 53;

const int Shoulder3Limit = 28;//30;  //Shoulder limit switch for the 3 leg position
const int Shoulder2Limit = 29; //31;  //Shoulder limit swtich for the 2 leg position
const int Center3Limit = 24;
const int Center2Limit = 25;
const int Ankle3LeftLimit = 30; //28;
const int Ankle2LeftLimit = 31; //29;
const int Ankle3RightLimit = 26; //26;
const int Ankle2RightLimit = 27; //27;

bool Shoulder3LimitTriggered = false;  //Boolean variables to minimize false reporting from the limit switches
bool Shoulder2LimitTriggered = false;
bool Center3LimitTriggered = false;
bool Center2LimitTriggered = false;
bool Ankle3LeftLimitTriggered = false;
bool Ankle2LeftLimitTriggered = false;
bool Ankle3RightLimitTriggered = false;
bool Ankle2RightLimitTriggered = false;
//bool WiggleStartTriggered = false;
//bool PeekStartTriggered = false;
//bool PeekEndTriggered = false;
bool NodTriggered = false;
bool CenterLegTestTriggered = false;
//bool ChuckleTriggered = false;
bool KneelTriggered = false;
//bool SadTriggered = false;
bool PeekTriggered = false;

unsigned long TimeOutValue = 25000;  //Motor timeout value. Was 21000  Acts as a kill switch at the end of the sequence
long TransitionStarted;              //Time count from initiating transition or animation
int state = 0;                       //Initial Start up state and idle state after timeout

void setPWMfrequency(int freq) {  //I upped the frequency of the PWM signal to quiet the motors down.
  TCCR0B = TCCR0B & 0b11111000 | freq;
  TCCR1B = TCCR1B & 0b11111000 | freq;
  TCCR2B = TCCR2B & 0b11111000 | freq;
  TCCR3B = TCCR3B & 0b11111000 | freq;
  TCCR4B = TCCR4B & 0b11111000 | freq;
}

//Motor Identification
//Direction indicators assume
//you are looking at the gear
//side of the motor.
//
//  Shoulder Motor     R=A
//                     L=B
//
//  Center Leg Motor   R=C
//                     L=D
//
//  Left Ankle Motor   R=E
//                     L=F
//
//  Right Ankle Motor  R=G
//                     L=H

void MotorActiveStatus(char Side, boolean s) {  //This is just shorthand to make the code simpler to write
  boolean state = s;
  if (Side == 'B') {
    digitalWrite(R_ENS, s);
  }
  if (Side == 'A') {
    digitalWrite(L_ENS, s);
  }
  if (Side == 'D') {
    digitalWrite(R_ENC, s);
  }
  if (Side == 'C') {
    digitalWrite(L_ENC, s);
  }
  if (Side == 'F') {
    digitalWrite(R_ENL, s);
  }
  if (Side == 'E') {
    digitalWrite(L_ENL, s);
  }
  if (Side == 'H') {
    digitalWrite(R_ENR, s);
  }
  if (Side == 'G') {
    digitalWrite(L_ENR, s);
  }
}
void setMotor(char side, byte pwm) {
  if (side == 'B') {
    analogWrite(RPWMS, pwm);
  }
  if (side == 'A') {
    analogWrite(LPWMS, pwm);
  }
  if (side == 'D') {
    analogWrite(RPWMC, pwm);
  }
  if (side == 'C') {
    analogWrite(LPWMC, pwm);
  }
  if (side == 'F') {
    analogWrite(RPWML, pwm);
  }
  if (side == 'E') {
    analogWrite(LPWML, pwm);
  }
  if (side == 'H') {
    analogWrite(RPWMR, pwm);
  }
  if (side == 'G') {
    analogWrite(LPWMR, pwm);
  }
}
void setup() {
  setPWMfrequency(0x02);  // timers at 7.81KHz once again to quiet motors

  pinMode(TWOLEG, INPUT);
  pinMode(THREELEG, INPUT);
  pinMode(CENTERLEGTEST, INPUT);
  pinMode(REARWHEELLIFT, INPUT);
  pinMode(PEEKESP, INPUT);
  pinMode(PEEK, INPUT);
  pinMode(WIGGLEESP, INPUT);
  pinMode(WIGGLE, INPUT);
  pinMode(CHUCKLEESP, INPUT);
  pinMode(CHUCKLE, INPUT);
  pinMode(KNEELESP, INPUT);
  pinMode(KNEEL, INPUT);
  pinMode(SADESP, INPUT);
  pinMode(SAD, INPUT);
  pinMode(STANDUPESP, INPUT);
  pinMode(STANDUP, INPUT);
  pinMode(NODESP, INPUT);
  pinMode(NOD, INPUT);
  pinMode(REARWHEELLIFT, INPUT);


  pinMode(RPWMS, OUTPUT);  //Shoulder
  pinMode(LPWMS, OUTPUT);
  pinMode(L_ENS, OUTPUT);
  pinMode(R_ENS, OUTPUT);

  pinMode(RPWMC, OUTPUT);  //Center leg
  pinMode(LPWMC, OUTPUT);
  pinMode(L_ENC, OUTPUT);
  pinMode(R_ENC, OUTPUT);

  pinMode(RPWML, OUTPUT);  //Left ankle
  pinMode(LPWML, OUTPUT);
  pinMode(L_ENL, OUTPUT);
  pinMode(R_ENL, OUTPUT);

  pinMode(RPWMR, OUTPUT);  //Right ankle
  pinMode(LPWMR, OUTPUT);
  pinMode(L_ENR, OUTPUT);
  pinMode(R_ENR, OUTPUT);

  delay(80000);  //Just a brief pause for boot of the control ESP32

  MotorActiveStatus('A', true);
  MotorActiveStatus('B', true);
  MotorActiveStatus('C', true);
  MotorActiveStatus('D', true);
  MotorActiveStatus('E', true);
  MotorActiveStatus('F', true);
  MotorActiveStatus('G', true);
  MotorActiveStatus('H', true);

  Serial.begin(9600);
  Serial.println("Maxstang's 232 Driver Board Initialized and Ready!");
}
void loop() {
  if (state == 0) {

    if (digitalRead(THREELEG) == HIGH) {
      Serial.println("Begin two leg to three leg transition. Deploy center leg.");
      TransitionStarted = millis();
      setMotor('D', 250);  //Center leg deploy
      state = 1;
    }

    if (digitalRead(TWOLEG) == HIGH) {
      Serial.println("Begin three leg to two leg transition. Rotate ankles and shoulder. ");
      TransitionStarted = millis();
      setMotor('A', 150);  //Shoulder rotate to 2 leg position 170
      setMotor('F', 150);  //Rotate ankles to 2 leg position 170
      setMotor('H', 150);
      state = 2;
    }
    
    if (digitalRead(CENTERLEGTEST) == HIGH && digitalRead(Center2Limit) == HIGH) {
      Serial.println("Deploy center leg for testing purposes");
      TransitionStarted = millis();
      setMotor('D', 190);  //Center leg deploy low speed
      state = 9;
    }

    if (digitalRead(CENTERLEGTEST) == HIGH && digitalRead(Center2Limit) == LOW && digitalRead(Center3Limit) == HIGH) {
      Serial.println("Retract testing leg");
      TransitionStarted = millis();
      setMotor('C', 190);  //Lift Center leg
      state = 10;
    }
    if (digitalRead(REARWHEELLIFT) == HIGH
      && digitalRead(Center3Limit) == HIGH
      && digitalRead(Ankle3LeftLimit) == HIGH){
      Serial.println("Lift rear wheels for speed");
      TransitionStarted = millis();
      setMotor('F', 100);  //Rotate ankle motors slightly lifting rear wheels
      setMotor('H', 100);
      delay(2000);
      setMotor('F', 0);  //Stop ankle motors
      setMotor('H', 0);
      state = 0;
      }
    if ((digitalRead(KNEEL) == HIGH || digitalRead(KNEELESP) == HIGH) //(digitalRead(KNEEL) == HIGH || 
       //&& KneelTriggered == false 
       && digitalRead(Shoulder3Limit) == HIGH 
       && digitalRead(Ankle3LeftLimit) == HIGH 
       && digitalRead(Center3Limit) == HIGH) {
      //KneelTriggered = true;
      Serial.println("KNEEL Sequence");
      TransitionStarted = millis();
      Serial.println("Retract center leg");
      setMotor('C', 80);  //Lift Center leg slowly
      delay(8000);        //
      setMotor('C', 0);   //Stop center leg
      
    }

    /*if (digitalRead(KNEELESP) == HIGH //(digitalRead(KNEEL) == HIGH || 
      && KneelTriggered == true
      && digitalRead(Shoulder3Limit) == HIGH 
      && digitalRead(Ankle3LeftLimit) == HIGH
      && digitalRead(Center3Limit) == LOW) {
      KneelTriggered = false;
      Serial.println("Reverse Kneel sequence!");
      TransitionStarted = millis();
      setMotor('D', 130);  //Extend center leg (sped up cause she was timing out) 
      state = 11;
    }
    */
    if ((digitalRead(NOD) == HIGH  || digitalRead(NODESP) == HIGH) //(digitalRead(NOD) == HIGH  || 
      && digitalRead(Center3Limit) == HIGH
      && digitalRead(Ankle3LeftLimit) == HIGH) {
      Serial.println("Nod sequence!");
      //NodTriggered = true;
      setMotor('C', 150);  //Center Leg Retract
      delay(1000);         //
      setMotor('C', 0);    //Stop Center Leg Retract
      delay(1000);
      setMotor('D', 150);  //Center Leg Extend
      delay(1000);
      setMotor('D', 0);  //Stop Center Leg Extend
      delay(1000);
      setMotor('C', 150);  //Center Leg Retract
      delay(1000);
      setMotor('C', 0);  //Stop Center Leg Retract
      delay(1000);
      setMotor('D', 150);  //Center Leg Extend
      state = 11;
    }
    
    if ((digitalRead(SAD) == HIGH  || digitalRead(SADESP) == HIGH) //(digitalRead(SAD) == HIGH  ||  
      && digitalRead(Center3Limit) == HIGH
      && digitalRead(Ankle3LeftLimit) == HIGH){
      Serial.println("Sad sequence!");
      setMotor('C', 100);  //Center Leg Retract
      delay(2000);         //
      setMotor('C', 0);    //Stop Center Leg Retract
      
    }
    
    if ((digitalRead(WIGGLE) == HIGH  || digitalRead(WIGGLEESP) == HIGH) //(digitalRead(WIGGLE) == HIGH  || 
      && digitalRead(Center3Limit) == HIGH
      && digitalRead(Ankle3LeftLimit) == HIGH) {
      //WiggleStartTriggered = true; Why is this commented out?
      Serial.println("Wiggle sequence!");
      setMotor('F', 220);  //Lift left wheel
      delay(1500);         //
      setMotor('F', 0);    //Stop left wheel lift
      delay(500);
      setMotor('E', 220);  //Drop left wheel
      delay(1500);
      setMotor('E', 0);  //Stop left wheel drop
      delay(500);
      setMotor('H', 220);  //Lift right wheel
      delay(1500);
      setMotor('H', 0);  //Stop right wheel Lift
      delay(500);
      setMotor('G', 220);  //Drop right wheel
      delay(1500);
      setMotor('G', 0);  //Stop right wheel drop
      delay(500);
      setMotor('F', 220);  //Lift left wheel
      delay(1500);
      setMotor('F', 0);  //Stop left wheel lift
      delay(500);
      setMotor('E', 220);  //Drop left wheel
      delay(1500);
      setMotor('E', 0);  //Stop left wheel drop
      delay(500);
      setMotor('H', 220);  //Lift right wheel
      delay(1500);
      setMotor('H', 0);  //Stop right wheel Lift
      delay(500);
      setMotor('G', 220);  //Drop right wheel
      delay(1500);
      setMotor('G', 0);  //Stop right wheel drop
      delay(500);
      setMotor('F', 220);  //Lift left wheel
      delay(1500);
      setMotor('F', 0);  //Stop left wheel lift
      delay(500);
      setMotor('E', 220);  //Drop left wheel
      delay(1500);
      setMotor('E', 0);  //Stop left wheel drop
      delay(500);
      setMotor('F', 220);  //Lift left wheel to reset ankle angle
      setMotor('H', 220);  //Lift right wheel to reset ankle angle
      delay(2000);         //
      setMotor('F', 0);    //Stop left wheel lift
      setMotor('H', 0);    //Stop left wheel lift
      delay(3000);
      TransitionStarted = millis();
      setMotor('E', 80);  //Drop left wheel
      setMotor('G', 80);  //Drop right wheel
      state = 8;
    }
    
    if ((digitalRead(CHUCKLE) == HIGH || digitalRead(CHUCKLEESP) == HIGH) //(digitalRead(CHUCKLE) == HIGH || d
      && digitalRead(Center3Limit) == HIGH
      && digitalRead(Ankle3LeftLimit) == HIGH) {
      Serial.println("Chuckle sequence!");
      setMotor('C', 150);  //Center Leg Retract
      delay(750);
      setMotor('C', 0);  //Stop Center Leg Retract
      delay(750);
      setMotor('C', 150);  //Center Leg Retract
      delay(750);
      setMotor('C', 0);  //Stop Center Leg Retract
      delay(750);
      setMotor('C', 150);  //Center Leg Retract
      delay(750);
      setMotor('C', 0);  //Stop Center Leg Retract
      delay(750);
      setMotor('C', 150);  //Center Leg Retract
      delay(750);
      setMotor('C', 0);  //Stop Center Leg Retract
      delay(750);
      setMotor('C', 150);  //Center Leg Retract
      delay(750);
      setMotor('C', 0);  //Stop Center Leg Retract
      delay(4000);
      TransitionStarted = millis();
      setMotor('D', 150);  //Center leg deploy
      state = 11;
    }
    
    if ((digitalRead(PEEK) == HIGH || digitalRead(PEEKESP) == HIGH) //(digitalRead(PEEK) == HIGH || 
      && PeekTriggered == false
      && digitalRead(Center3Limit) == HIGH
      && digitalRead(Ankle3LeftLimit) == HIGH) {
      PeekTriggered = true;
      Serial.println("Peek sequence!");
      TransitionStarted = millis();
      setMotor('F', 80);  //Lift left wheel
      setMotor('H', 80);  //Lift right wheel
      delay(9000);        //
      setMotor('F', 0);   //Stop left wheel lift
      setMotor('H', 0);   //Stop right wheel Lift
      
    }
    
    if ((digitalRead(PEEK) == HIGH || digitalRead(PEEKESP) == HIGH) //(digitalRead(PEEK) == HIGH || 
      && PeekTriggered == true
      && digitalRead(Center3Limit) == HIGH
      && digitalRead(Ankle3LeftLimit) == LOW) {
      PeekTriggered = false;
      Serial.println("Reverse Peek sequence!");
      TransitionStarted = millis();
      setMotor('E', 80);  //Drop left wheel
      setMotor('G', 80);  //Drop right wheel
      state = 8;
    }
    
    if ((digitalRead(STANDUP) == HIGH || digitalRead(STANDUPESP) == HIGH) //(digitalRead(STANDUP) == HIGH || 
      && digitalRead(Shoulder3Limit) == HIGH
      && digitalRead(Ankle3LeftLimit) == HIGH
      && digitalRead(Center3Limit) == LOW){
      Serial.println("Stand Back Up!");
      KneelTriggered == false; 
      TransitionStarted = millis();
      setMotor('D', 200);  //Extend center leg
      state = 11;
    }
  }

  if (state == 1) {

    if (Center3LimitTriggered == false && digitalRead(Center3Limit) == HIGH) {
      Center3LimitTriggered = true;
      Serial.println("Center leg motor locked in 3 leg postion. Rotate shoulder and ankles.");
      setMotor('D', 0);    //Stop center leg motor
      delay(2000);         //Delay to let things settle
      setMotor('B', 170);  //Rotate shoulder to 3 leg position 140
      setMotor('E', 125);  //Rotate ankles to 3 leg postion
      setMotor('G', 125);
      state = 5;
    }
  }

  if (state == 2) {

    if (Shoulder2LimitTriggered == false && digitalRead(Shoulder2Limit) == HIGH) {
      Shoulder2LimitTriggered = true;
      setMotor('A', 0);  //Stop shoulder motor
      Serial.println("Shoulder locked in two leg position.");
    }

    if (Ankle2LeftLimitTriggered == false && digitalRead(Ankle2LeftLimit) == HIGH) {
      Ankle2LeftLimitTriggered = true;
      Serial.println("Left ankle locked in two leg position.");
      setMotor('F', 0);  //Stop left ankle motor
    }

    if (Ankle2RightLimitTriggered == false && digitalRead(Ankle2RightLimit) == HIGH) {
      Ankle2RightLimitTriggered = true;
      Serial.println("Right ankle locked in two leg position.");
      setMotor('H', 0);  //Stop right ankle motor
    }
    if (Shoulder2LimitTriggered == true && Ankle2LeftLimitTriggered == true && Ankle2RightLimitTriggered == true) {
      state = 3;  //Make sure both shoulder and ankle are stopped before proceeding.
    }
  }

  if (state == 3) {
    delay(4000);  //Delay for 1/4th second to let momentum settle, but this may be too long.  
    Serial.println("Begin center leg retraction.");
    setMotor('C', 250);  //Retract center leg.
    state = 4;
  }

  if (state == 4) {
    if (Center2LimitTriggered == false && digitalRead(Center2Limit) == HIGH) {
      Center2LimitTriggered = true;
      Serial.println("Center leg retracted. Transition complete.");
      setMotor('C', 0);  //Stop Center leg
      
    }
  }

  if (state == 5) {
    if (Shoulder3LimitTriggered == false && digitalRead(Shoulder3Limit) == HIGH) {
      Shoulder3LimitTriggered = true;
      Serial.println("Shoulder motor locked in 3 leg postion.");
      setMotor('B', 0);  //Stop shoulder motor
    }
    if (Ankle3LeftLimitTriggered == false && digitalRead(Ankle3LeftLimit) == HIGH) {
      Ankle3LeftLimitTriggered = true;
      Serial.println("Left ankle motor locked in 3 leg postion.");
      setMotor('E', 0);  //Stop left ankle motor
    }
    if (Ankle3RightLimitTriggered == false && digitalRead(Ankle3RightLimit) == HIGH) {
      Serial.println("Right ankle motor locked in 3 leg postion.");
      setMotor('G', 0);  //Stop right ankle motor
    }

    if (Shoulder3LimitTriggered == true && Ankle3RightLimitTriggered == true && Ankle3RightLimitTriggered == true) {
      Serial.println("Transition complete.");
      
    }
  }

  if (state == 6) {
    if (Center3LimitTriggered == false && digitalRead(Center3Limit) == HIGH) {
      Center3LimitTriggered = true;
      setMotor('D', 0);  //Stop center leg motor
      Serial.println("Center leg motor locked in 3 leg postion.");
    }

    if (Shoulder3LimitTriggered == false && digitalRead(Shoulder3Limit) == HIGH) {
      Shoulder3LimitTriggered = true;
      setMotor('B', 0);  //Stop shoulder motor
      Serial.println("Shoulder motor locked in 3 leg postion.");
    }

    if (Center3LimitTriggered == true && Shoulder3LimitTriggered == true) {
      state = 8;
    }
  }

  if (state == 8) {
    if (Ankle3LeftLimitTriggered == false && digitalRead(Ankle3LeftLimit) == HIGH) {
      Ankle3LeftLimitTriggered = true;
      Serial.println("Left ankle locked in three leg position.");
      setMotor('E', 0);  //Stop left ankle
    }

    if (Ankle3RightLimitTriggered == false && digitalRead(Ankle3RightLimit) == HIGH) {
      Ankle3RightLimitTriggered = true;
      Serial.println("Right ankle locked in three leg position.");
      setMotor('G', 0);  //Stop right ankle
    }

    if (Ankle3RightLimitTriggered == true && Ankle3RightLimitTriggered == true) {
      Serial.println("Transition complete.");
      
    }
  }

  if (state == 9) {
    if (CenterLegTestTriggered == false && digitalRead(Center3Limit) == HIGH) {
      CenterLegTestTriggered = true;
      setMotor('D', 0);  //Stop center leg motor
      Serial.println("Center leg deployed to test position.");
      
    }
  }

  if (state == 10) {
    if (digitalRead(Center2Limit) == HIGH) {
      //CenterLegTestTriggered = true;
      setMotor('C', 0);  //Stop center leg motor
      Serial.println("Center leg test completed");
      
    }
  }

  if (state == 11) {
    if (Center3LimitTriggered == false && digitalRead(Center3Limit) == HIGH) {
      Center3LimitTriggered = true;
      Serial.println("Center leg motor locked in 3 leg postion.");
      setMotor('D', 0);  //Stop center leg motor
      
    }
  }

  if (((millis() - TransitionStarted) > TimeOutValue) && (state != 0)) {
    Serial.println("Motor Timeout");  //Timeout the motors if any are still trying to turn AND set state back to 0.
    setMotor('A', 0);
    setMotor('B', 0);
    setMotor('C', 0);
    setMotor('D', 0);
    setMotor('E', 0);
    setMotor('F', 0);
    setMotor('G', 0);
    setMotor('H', 0);
    Shoulder3LimitTriggered = false;  //Set trigger boolean values back to false
    Shoulder2LimitTriggered = false;
    Center3LimitTriggered = false;
    Center2LimitTriggered = false;
    Ankle3LeftLimitTriggered = false;
    Ankle2LeftLimitTriggered = false;
    Ankle3RightLimitTriggered = false;
    Ankle2RightLimitTriggered = false;
    //PeekStartTriggered = false;
    //PeekEndTriggered == false;
    //WiggleStartTriggered = false;
    //NodTriggered = false;
    //CenterLegTestTriggered = false;
    state = 0;
  }
}
