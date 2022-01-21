#include <Arduino.h>

#include <IRremote.h>
#include <Servo.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


#include <math.h>

#include "buttons.h"
#include "types.h"
#include "pitches.h"

#define ID_Rotator_LF 0
#define ID_Lift_LF    1
#define ID_Kick_LF    2

#define ID_Rotator_RF 4
#define ID_Lift_RF    5
#define ID_Kick_RF    6

#define ID_Rotator_LB 8
#define ID_Lift_LB    9
#define ID_Kick_LB    10

#define ID_Rotator_RB 12
#define ID_Lift_RB    13
#define ID_Kick_RB    14

#define ID_Neck 15

const int SLEEP =  1;
const int SLEEP_LOOP = 2;
const int AWAKE = 3;
const int AWAKE_LOOP = 4;
const int WALK = 5;
const int WALK_LOOP = 6;
const int PACE = 7;
const int PACE_LOOP  = 8;


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50

/* PIN LAYOUT
 * 
 * I2C
 *  - SDA : A4
 *  - SCL : A5
 *  - GRD : GRD
 *  - VCC : 5V
 * 
 */

 /* SERVO LIMIT NOTES
 * Normal  : range(90, 470)
 * Serve 14: range(120, 400)
 * 
 * B LEGS NEED LIFT + KICK REVERSED
 * 
 * rotator reverse 4 and 8
 *     
 */



typedef struct {
  float x;
  float y;
  float z;
} Vector3;

typedef struct {
  float d;
  float z;
} Vector2;


// ----------------------------- MELODIES ----------------------------- \\ 


// Melody data
int lengths_awake [16] = {NOTE_DS6, NOTE_DS5, NOTE_AS5, NOTE_GS5, NOTE_DS5, NOTE_DS6, NOTE_AS5};
int notes_awake [16] = {250, 125, 375, 250, 250, 250, 750};
Melody mel_awake = {{notes_awake}, {lengths_awake}, 7};


int lengths_sleep [16] = {NOTE_GS5, NOTE_DS5, NOTE_GS4, NOTE_AS4};
int notes_sleep [16] = {500, 500, 500, 500};
Melody mel_sleep = {{notes_sleep}, {lengths_sleep}, 4};

Melody mel_1 = {{NOTE_C4}, {750}, 1};
Melody mel_2 = {{NOTE_D4}, {750}, 1};
Melody mel_3 = {{NOTE_E4}, {750}, 1};
Melody mel_4 = {{NOTE_F4}, {750}, 1};
Melody mel_5 = {{NOTE_G4}, {750}, 1};
Melody mel_6 = {{NOTE_A5}, {750}, 1};
Melody mel_7 = {{NOTE_B5}, {750}, 1};
Melody mel_8 = {{NOTE_C5}, {750}, 1};
Melody mel_9 = {{NOTE_D4}, {750}, 1};

// play to stop current melody
Melody mel_0 = {};
// Play Melodies

Melody mel_active = {};

int next_note_time = 0;
int current_note = 0;

void init_melody(Melody mel){
  mel_active = mel;
  next_note_time = 0;
  current_note = 0;
}

#define SPEAKER_PIN 8
void play_melody(){
  if(current_note < mel_active.number_of_notes){
    int current_time = millis();
    Serial.println(String(current_note) + ' ' + String(current_time) + ' ' + String(next_note_time));
    
    if(current_time >= next_note_time){
      tone(8, mel_active.notes[current_note]);
      current_note ++;
    }
    
  }
  else{
    int pin = SPEAKER_PIN;
    noTone(pin);
  }
}


// ----------------------------- INFRA-RED ----------------------------- \\ 



const int RECV_PIN = 4;
IRrecv irrecv(RECV_PIN);
decode_results ir_results;

long current_ir_value = 0;
long last_ir_value = 0;

long ir_value = 0;

void readIR(){
  irrecv.decode(&ir_results);
  irrecv.resume();
  
  // Trigger when down
  //  set ir_value to 0 to trigger only on down
  if(ir_results.value != IR_REDO && ir_results.value != 0){
    ir_value = ir_results.value;
  }
  else if (ir_results.value == 0){
    ir_value = 0;
  }
  /*
  // Trigger on click only
  if(ir_results.value != IR_REDO && ir_results.value != 0){
    ir_value = ir_results.value;
  }
  else{
    ir_value = 0;
  }
  */
  
  ir_results.value = 0;
}

// ----------------------------- SERVOS ----------------------------- \\ 

int Neck_Position = 280;


// Parameters
const RobotState lower_limit = {{470, 90, 120}, {90, 470, 400}, {90, 470, 380}, {470, 90, 120}, 280};
const RobotState upper_limit = {{90, 470, 400}, {470, 90, 120}, {470, 90, 140}, {90, 470, 400}, 280};

const RobotState fine_offset = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};

// Configurations

RobotState spider = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};


RobotState cfg_start = {{512, 512, 512}, {312, 712, 512}, {712, 712, 512}, {512, 512, 512}, 280};
RobotState cfg_big = {{512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 280};

// Animation Data
#define FRAMES_PER_ANIMATIONS 4
 
  // walk
RobotState anim_walk_1 = {{712, 712, 512}, {512, 512, 512}, {512, 512, 512}, {712, 512, 512}, 280};
RobotState anim_walk_2 = {{712, 512, 512}, {512, 512, 512}, {712, 512, 512}, {512, 712, 512}, 280}; 
RobotState anim_walk_3 = {{512, 512, 512}, {712, 712, 512}, {712, 512, 512}, {512, 512, 512}, 280}; 
RobotState anim_walk_4 = {{512, 512, 512}, {712, 512, 512}, {512, 712, 512}, {712, 512, 512}, 280}; 

Animation anim_walk = {{anim_walk_1, anim_walk_2, anim_walk_3, anim_walk_4}, 200, true};

  // pace
RobotState anim_pace_1 = {{712, 712, 512}, {512, 512, 512}, {712, 512, 512}, {512, 712, 512}, 280};
RobotState anim_pace_2 = {{712, 512, 512}, {512, 512, 512}, {712, 512, 512}, {512, 512, 512}, 280}; 
RobotState anim_pace_3 = {{512, 512, 512}, {712, 712, 512}, {512, 712, 512}, {712, 512, 512}, 280}; 
RobotState anim_pace_4 = {{512, 512, 512}, {712, 512, 512}, {512, 512, 512}, {712, 512, 512}, 280}; 

Animation anim_pace = {{anim_pace_1, anim_pace_2, anim_pace_3, anim_pace_4}, 100, true};

  // trot
RobotState anim_trot_1 = {{712, 712, 512}, {512, 512, 512}, {512, 712, 512}, {712, 512, 512}, 280};
RobotState anim_trot_2 = {{712, 512, 512}, {512, 512, 512}, {512, 512, 512}, {712, 512, 512}, 280}; 
RobotState anim_trot_3 = {{512, 512, 512}, {712, 712, 512}, {712, 512, 512}, {512, 712, 512}, 280}; 
RobotState anim_trot_4 = {{512, 512, 512}, {712, 512, 512}, {712, 512, 512}, {512, 512, 512}, 280}; 

Animation anim_trot = {{anim_trot_1, anim_trot_2, anim_trot_3, anim_trot_4}, 50, true};

// Play Animations

Animation current_animation = {};
int current_frame_index = 0;
int next_frame_time = 0;

void SetAnimation(Animation new_animation, int non){
  current_animation = new_animation;

  current_frame_index = 0;
  next_frame_time = 0;
}

void play_animation(){
  int current_time = millis();

  if(current_frame_index == 4 && !current_animation.is_looping){
      return;
  }
  else if(current_frame_index == 4 && current_animation.is_looping){
    current_frame_index = 0;
  }
  
  if(current_time >= next_frame_time){
      spider.LeftFront  = current_animation.frames[current_frame_index].LeftFront;
      spider.RightFront = current_animation.frames[current_frame_index].RightFront;
      spider.LeftBack   = current_animation.frames[current_frame_index].LeftBack;
      spider.RightBack  = current_animation.frames[current_frame_index].RightBack;
      
      current_frame_index += 1;
      next_frame_time = current_time + current_animation.frame_delay;
  }
}

// ------------------------------- SETUP  ------------------------------- \\ 

void setup() {
  Serial.begin(9600);
Serial.println("Started");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  irrecv.enableIRIn();
  irrecv.blink13(true);

  Serial.println("Started");
  //Serial.println(start_up_melody[0].duration);
  //initalize_melody(start_up_melody);

  spider = cfg_start;
  
  //SetAnimation(anim_start);
}


// ----------------------------- MAIN LOOP ----------------------------- \\ 


String command = "";
long state = SLEEP;
void loop() {
  /*
  if(Serial.available()){
    command = Serial.readStringUntil(' ');
    command.trim();
    Serial.print("Command recived: ");
    Serial.println(command);
    if(command.startsWith("move")){
      String leg = Serial.readStringUntil(' ');
      int rotate = Serial.readStringUntil(' ').toInt();
      int lift = Serial.readStringUntil(' ').toInt();
      int kick = Serial.readStringUntil(' ').toInt();
     
      //set_leg(leg, rotate, lift, kick);
    }
    else if(command.startsWith("set")){
      int id = Serial.readStringUntil(' ').toInt();
      int freq = Serial.readStringUntil(' ').toInt();

      Serial.println("Moving " + String(id) + " to " + String(freq));
      //write_servo(id, freq);
    }
    else if(command.startsWith("config")){
      String config_mode = Serial.readStringUntil(' ');
      config_mode.trim();
      if(config_mode == "start"){
        Serial.println("Going to start config");
        spider = cfg_start;
      }
      else if(config_mode == "big"){
        spider = cfg_big;
      }
      
    }
  }
  */
  //Serial.println(state);
  // STATE MACHINE
  // ----  SLEEP  ---- //
  if(state == SLEEP){
    current_animation = anim_walk;
    state = SLEEP_LOOP;
    init_melody(mel_sleep);
    Serial.println("sleep");
  }
  else if(state == SLEEP_LOOP){
    // EXIT CASES
    if(ir_value == IR_POWER){
      state = AWAKE;
    }
  }
  
  // ----  AWAKE  ---- //
  else if(state == AWAKE){
    current_animation = anim_walk;
    state = AWAKE_LOOP;
    init_melody(mel_awake);
    Serial.println("awake");
  }
  else if(state == AWAKE_LOOP){
    // EXIT CASES
    if(ir_value == IR_POWER){
      state = SLEEP;
    }
    else if(ir_value == IR_PLAY){
      state = WALK;
    }
  }
  
  // ----  WALK  ---- //
  else if(state == WALK){
    current_animation = anim_walk;
    state = WALK_LOOP;
  }
  else if(state == WALK_LOOP){
    current_animation = anim_walk;
    
    // IR FUNCTIONS + EXIT CASES
    if(ir_value == IR_FFWD){
      state = PACE;
    }
    else if(ir_value == IR_RWD){
      // PLAY ERROR SOUND
    }
    else if(ir_value == IR_POWER){
      state = SLEEP;
    }
    else if(ir_value == IR_PLAY){
      state = AWAKE;
    }
  }
  
  // ----  PACE  ---- //
  else if(state == PACE){
    current_animation = anim_pace;
    state = PACE_LOOP;
  }
  else if(state == PACE_LOOP){
    // IR FUNCTIONS + EXIT CASES
    if(ir_value == IR_RWD){
      //Serial.println("going back");
      state = WALK;
    }
    else if(ir_value == IR_FFWD){
      // PLAY ERROR SOUND
    }
    else if(ir_value == IR_POWER){
      state = SLEEP;
    }
    else if(ir_value == IR_PLAY){
      state = AWAKE;
    }
  }
  
  play_animation();
  write_servos();
  readIR();
  //Serial.println(ir_value);
  play_melody();
  delay(200);
}

//https://arduino.stackexchange.com/questions/1013/how-do-i-split-an-incoming-string

// ----------------------------- UTILITY ----------------------------- \\ 

void print_robot_state(){
  Serial.println("Spider State \n-----------------------------------");
  
  Serial.print("Left Front: ");
  Serial.print("Roator - " + String(spider.LeftFront.Rotator) + " ");
  Serial.print("Lift - " + String(spider.LeftFront.Lift) + " ");
  Serial.println("Kick - " + String(spider.LeftFront.Kick)+ " ");

  Serial.print("Right Front: ");
  Serial.print("Roator - " + String(spider.RightFront.Rotator) + " ");
  Serial.print("Lift - " + String(spider.RightFront.Lift)+ " ");
  Serial.println("Kick - " + String(spider.RightFront.Kick)+ " ");
  
  Serial.print("Left Back: ");
  Serial.print("Roator - " + String(spider.LeftBack.Rotator) + " ");
  Serial.print("Lift - " + String(spider.LeftBack.Lift)+ " ");
  Serial.println("Kick - " + String(spider.LeftBack.Kick) + " ");

  Serial.print("Left Back: ");
  Serial.print("Roator - " + String(spider.RightBack.Rotator) + " ");
  Serial.print("Lift - " + String(spider.RightBack.Lift) + " ");
  Serial.println("Kick - " + String(spider.RightBack.Kick) + " ");
}

// ----------------------------- SERVO CONTROL ----------------------------- \\ 

void set_leg(String leg, int Rotator, int Lift, int Kick){
  if(leg == "LF") {
      spider.LeftFront = {Rotator, Lift, Kick};
  }
  else if(leg == "LB") {
      spider.LeftBack = {Rotator, Lift, Kick};
  }
  else if(leg == "RF") {
      spider.RightFront = {Rotator, Lift, Kick};
  }
  else if(leg == "RB") {
      spider.RightFront = {Rotator, Lift, Kick};
  }
  else {
      Serial.println("Bad Input");
  }
  

  print_robot_state();
}

void write_servo(int id, int freq){
  pwm.setPWM(id, 0, freq);
}

void write_servos(){
  pwm.setPWM(ID_Rotator_LF, 0, map(spider.LeftFront.Rotator + fine_offset.LeftFront.Rotator, 0, 1024,   lower_limit.LeftFront.Rotator,  upper_limit.LeftFront.Rotator));
  pwm.setPWM(ID_Lift_LF,    0, map(spider.LeftFront.Lift    + fine_offset.LeftFront.Lift, 0, 1024,      lower_limit.LeftFront.Lift,     upper_limit.LeftFront.Lift   ));
  pwm.setPWM(ID_Kick_LF,    0, map(spider.LeftFront.Kick    + fine_offset.LeftFront.Kick, 0, 1024,      lower_limit.LeftFront.Kick,     upper_limit.LeftFront.Kick   ));

  pwm.setPWM(ID_Rotator_LB, 0, map(spider.LeftBack.Rotator + fine_offset.LeftBack.Rotator, 0, 1024,     lower_limit.LeftBack.Rotator,   upper_limit.LeftBack.Rotator));
  pwm.setPWM(ID_Lift_LB,    0, map(spider.LeftBack.Lift    + fine_offset.LeftBack.Lift, 0, 1024,        lower_limit.LeftBack.Lift,      upper_limit.LeftBack.Lift   ));
  pwm.setPWM(ID_Kick_LB,    0, map(spider.LeftBack.Kick    + fine_offset.LeftBack.Kick, 0, 1024,        lower_limit.LeftBack.Kick,      upper_limit.LeftBack.Kick   ));

  pwm.setPWM(ID_Rotator_RF, 0, map(spider.RightFront.Rotator + fine_offset.RightFront.Rotator, 0, 1024, lower_limit.RightFront.Rotator, upper_limit.RightFront.Rotator));
  pwm.setPWM(ID_Lift_RF,    0, map(spider.RightFront.Lift    + fine_offset.RightFront.Lift, 0, 1024,    lower_limit.RightFront.Lift,    upper_limit.RightFront.Lift   ));
  pwm.setPWM(ID_Kick_RF,    0, map(spider.RightFront.Kick    + fine_offset.RightFront.Kick, 0, 1024,    lower_limit.RightFront.Kick,    upper_limit.RightFront.Kick   ));

  pwm.setPWM(ID_Rotator_RB, 0, map(spider.RightBack.Rotator + fine_offset.RightBack.Rotator, 0, 1024,   lower_limit.RightBack.Rotator,  upper_limit.RightBack.Rotator));
  pwm.setPWM(ID_Lift_RB,    0, map(spider.RightBack.Lift    + fine_offset.RightBack.Lift, 0, 1024,      lower_limit.RightBack.Lift,     upper_limit.RightBack.Lift  ));
  pwm.setPWM(ID_Kick_RB,    0, map(spider.RightBack.Kick    + fine_offset.RightBack.Kick, 0, 1024,      lower_limit.RightBack.Kick,     upper_limit.RightBack.Kick  ));

  pwm.setPWM(ID_Neck,       0, map(spider.Neck + fine_offset.Neck, 0, 1024, lower_limit.Neck, upper_limit.Neck  ));

}
