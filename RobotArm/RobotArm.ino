#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50

/* PIN LAYOUT
 * 
 * Led
 *  Preset mode indicator: 11
 *  Manual mode indicator: 10
 * 
 * Buttons
 *  Toggle mode: 2
 *  Preset 1   : 3
 *  Preset 2   : 4
 *  Preset 3   : 5
 *  Preset 4   : 6
 * 
 * Dials
 *  Gripper input: A0
 *  Base input   : A1
 *  Forward input: A2
 *  Lift input   : A3
 * 
 * Servo Board
 *  Serial Data (SDA): A4
 *  Serial Clock(SCL): A5
 */

const int PresetLed = 11;
const int ManualLed = 10;

const int ForwardArmChannel = 2;
const int ForwardArmMax = 410;
const int ForwardArmMin = 160;

const int LiftArmChannel = 0;
const int LiftArmMax = 490;
const int LiftArmMin = 290;

const int GripperChannel = 3;
const int GripperMax = 500;
const int GripperMin = 280;

const int BaseChannel = 1;
const int BaseMax = 500;
const int BaseMin = 280;

const int SaveButton = 2;
const int PresetInputPins[] = {3, 4, 5, 6};

const int LiftInput = A3;
const int BaseInput = A2;
const int ForwardInput = A1;
const int GripperInput = A0;

typedef struct  {
  int ForwardPosition;
  int LiftPosition;
  int GripperPosition;
  int BasePosition;
} RobotState;

RobotState presetStates[] = {{0, 0, 0, 0}, {1, 1, 1, 1}, {2, 2, 2, 2}, {3, 3, 3, 3}};


void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pinMode(SaveButton, INPUT_PULLUP);
  digitalWrite(SaveButton, HIGH);
 
  for(int i = 0; i < sizeof(PresetInputPins)/sizeof(PresetInputPins[0]); i++){
    pinMode(PresetInputPins[i], INPUT_PULLUP);
    digitalWrite(PresetInputPins[i], HIGH);
  }

  attachInterrupt(digitalPinToInterrupt(SaveButton), ToggleState, FALLING);

  pinMode(LiftInput, INPUT);
  pinMode(BaseInput, INPUT);
  pinMode(ForwardInput, INPUT);
  pinMode(GripperInput, INPUT);
  digitalWrite(GripperInput, HIGH);

  pinMode(PresetLed, OUTPUT);
  pinMode(ManualLed, OUTPUT);
}

bool PresetControls = false;
void ToggleState(){
  Serial.print("Toggleing State");
  PresetControls = !PresetControls;
}

String command;
int ForwardArmPulseLength = 300;
int LiftArmPulseLength = 400;
int GripperPulseLength = 0;
int BasePulseLength = 0;

bool GripperOpen = false;
void loop() {


  // In preset control mode:
  // When a preset button is click 
  // Move robot to that state
  if(PresetControls){
    
    for(int i = 0; i != 4; i++){
      if(!digitalRead(PresetInputPins[i])){
        ForwardArmPulseLength = presetStates[i].ForwardPosition;
        LiftArmPulseLength = presetStates[i].LiftPosition;
        BasePulseLength = presetStates[i].BasePosition;
        GripperPulseLength = presetStates[i].GripperPosition;
      }
    }
  }
  // In normal control mode:
  // use pots to control robot
  // When a preset button is click 
  // Save the current state to the correct preset
  else{
    ForwardArmPulseLength = map(analogRead(ForwardInput), 0, 1023, ForwardArmMin, ForwardArmMax);
    LiftArmPulseLength = map(analogRead(LiftInput), 0, 1023, LiftArmMin, LiftArmMax);
    BasePulseLength = map(analogRead(BaseInput), 0, 1023, BaseMin, BaseMax);
    GripperPulseLength = map(analogRead(GripperInput), 0, 1023, GripperMin, GripperMax);

    for(int i = 0; i != 4; i++){
      if(!digitalRead(PresetInputPins[i])){
        Serial.print(String(i) + " updated: ");
        presetStates[i] = (RobotState){ForwardArmPulseLength, LiftArmPulseLength, GripperPulseLength, BasePulseLength};
      }
    }
  }

  //Serial.println("INPUTS:  " + String(analogRead(ForwardInput)) + " " + String(analogRead(LiftInput)) + " " + String(analogRead(BaseInput)) + " " + String(analogRead(GripperInput)));
  //Serial.println("LENGTHS: " + String(ForwardArmPulseLength)+ " " + String(LiftArmPulseLength)+ " " + String(BasePulseLength) + " " + String(GripperPulseLength));
  
  pwm.setPWM(ForwardArmChannel, 0, constrain(ForwardArmPulseLength, ForwardArmMin, ForwardArmMax));
  pwm.setPWM(LiftArmChannel, 0, constrain(LiftArmPulseLength, LiftArmMin, LiftArmMax));
  pwm.setPWM(GripperChannel, 0, constrain(GripperPulseLength, GripperMin, GripperMax));
  pwm.setPWM(BaseChannel, 0, constrain(BasePulseLength, BaseMin, BaseMax));
  delay(50);
  
  Serial.print(String(PresetControls) + ": ");
  Serial.print("LENGTHS: " + String(ForwardArmPulseLength)+ " " + String(LiftArmPulseLength)+ " " + String(BasePulseLength) + " " + String(GripperPulseLength));
  Serial.println();
  /*
  Serial.println(String(digitalRead(PresetInputPins[0])) + ' ' + 
                 String(digitalRead(PresetInputPins[1])) + ' ' + 
                 String(digitalRead(PresetInputPins[2])) + ' ' + 
                 String(digitalRead(PresetInputPins[3])) + ' ' + 
                 String(digitalRead(SaveButton)) + ' ' + 
                 String(PresetControls)
                 );
  */
  
  digitalWrite(PresetLed,  PresetControls);
  digitalWrite(ManualLed, !PresetControls);
}
