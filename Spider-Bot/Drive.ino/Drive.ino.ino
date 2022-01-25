#include <Arduino.h>

#define IR_USE_TIMER3
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


// ----------------------------- MELODIES ----------------------------- // 


// Melody data

Melody mel_awake = {{NOTE_DS6, NOTE_DS5, NOTE_AS5, NOTE_GS5, NOTE_DS5, NOTE_DS6, NOTE_AS5}, {250, 125, 375, 250, 250, 250, 750}, 7};
Melody mel_sleep = {{NOTE_GS5, NOTE_DS5, NOTE_GS4, NOTE_AS4}, {500, 500, 500, 500}, 4};
Melody mel_error = {{NOTE_C5, NOTE_A4}, {125, 125}, 2};
Melody mel_ping = {{NOTE_C5, NOTE_E5}, {125, 125}, 2};

// Play Melodies

Melody mel_active = {};

int next_note_time = 0;
int current_note = 0;

void init_melody(Melody mel){
  mel_active = mel;
  next_note_time = 0;
  current_note = 0;
  
  IrReceiver.stop();
}

#define SPEAKER_PIN 8
void play_melody(){
  long current_time = millis();
  if(current_time < next_note_time) return;
  
  if(current_note <= mel_active.number_of_notes){
    tone(SPEAKER_PIN, mel_active.notes[current_note]);
    next_note_time = current_time + mel_active.lengths[current_note];
    current_note ++;
  }
  else{
    noTone(SPEAKER_PIN);
    IrReceiver.start();
  }
}


// ----------------------------- INFRA-RED ----------------------------- //



const int RECV_PIN = 5;
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
  
  ir_results.value = 0;


  // play tones for user feedback
  if(ir_value == IR_POWER ||
     ir_value == 0) return;

  if(ir_value == IR_FOUR ||
     ir_value == IR_TWO ||
     ir_value == IR_SIX ||
     ir_value == IR_EIGHT ||
     ir_value == IR_PLAY){
     init_melody(mel_ping);
     return;
  }
  else{
    init_melody(mel_error);
     return;
  }
}

// ----------------------------- SERVOS ----------------------------- //

// Parameters
const RobotState lower_limit = {{470, 90, 120}, {90, 470, 400}, {90, 470, 380}, {470, 90, 120}, 280};
const RobotState upper_limit = {{90, 470, 400}, {470, 90, 120}, {470, 90, 140}, {90, 470, 400}, 280};

const RobotState fine_offset = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};

// Configurations

RobotState cfg_active = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};


const RobotState cfg_start = {{1024, 1024, 0}, {1024, 1024, 0}, {1024, 1024, 0}, {1024, 1024, 0}, 512};
const RobotState cfg_awake = {{512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}, 512};
const RobotState cfg_sleep = {{1024, 1024, 0}, {1024, 1024, 0}, {1024, 1024, 0}, {1024, 1024, 0}, 512};

// Animation 

  // awake
const RobotState cfg_wake_up_1 = {{1024, 1024, 512}, {1024, 1024, 512}, {1024, 1024, 512}, {1024, 1024, 512}, 512};
const RobotState cfg_wake_up_2 = {{512, 1024, 512},  {512, 1024, 512},  {512, 1024, 512},  {512, 1024, 512},  512};
const Animation anim_awake = {{cfg_start, cfg_wake_up_1, cfg_wake_up_2, cfg_awake}, {100, 200, 200, 200}, 4, false};

 // sleep
const Animation anim_sleep = {{cfg_awake, cfg_wake_up_2, cfg_wake_up_1,  cfg_sleep}, {200, 200, 200, 100}, 4, false};

const Animation anim_stand = {{cfg_awake}, {0}, 1, false};

  // walk f/b
//const RobotState anim_walk_fwd_1 = {{712, 712, 512}, {512, 512, 512}, {512, 512, 512}, {712, 512, 512}, 512};
//const RobotState anim_walk_fwd_2 = {{712, 512, 512}, {512, 512, 512}, {712, 512, 512}, {512, 712, 512}, 512}; 
//const RobotState anim_walk_fwd_3 = {{512, 512, 512}, {712, 712, 512}, {712, 512, 512}, {512, 512, 512}, 512}; 
//const RobotState anim_walk_fwd_4 = {{512, 512, 512}, {712, 512, 512}, {512, 712, 512}, {712, 512, 512}, 512}; 

const Animation anim_walk_fwd = {{{{712, 712, 512}, {512, 512, 512}, {512, 512, 512}, {712, 512, 512}, 512}, 
                                  {{712, 512, 512}, {512, 512, 512}, {712, 512, 512}, {512, 712, 512}, 512}, 
                                  {{512, 512, 512}, {712, 712, 512}, {712, 512, 512}, {512, 512, 512}, 512}, 
                                  {{512, 512, 512}, {712, 512, 512}, {512, 712, 512}, {712, 512, 512}, 512}}, 
                                 {200, 200, 200, 200}, 4, true};
                                 
const Animation anim_walk_bwd = {{{{512, 512, 512}, {712, 512, 512}, {512, 712, 512}, {712, 512, 512}, 512},
                                  {{512, 512, 512}, {712, 712, 512}, {712, 512, 512}, {512, 512, 512}, 512}, 
                                  {{712, 512, 512}, {512, 512, 512}, {712, 512, 512}, {512, 712, 512}, 512}, 
                                  {{712, 712, 512}, {512, 512, 512}, {512, 512, 512}, {712, 512, 512}, 512}}, 
                                 {200, 200, 200, 200}, 4, true};

  // walk l/r
const RobotState anim_walk_lft_1 = {{312, 712, 512}, {512, 512, 512}, {512, 512, 512}, {312, 512, 512}, 512};
const RobotState anim_walk_lft_2 = {{312, 512, 512}, {512, 512, 512}, {312, 512, 512}, {512, 712, 512}, 512}; 
const RobotState anim_walk_lft_3 = {{512, 512, 512}, {312, 712, 512}, {312, 512, 512}, {512, 512, 512}, 512}; 
RobotState anim_walk_lft_4 = {{512, 512, 512}, {312, 512, 512}, {512, 712, 512}, {312, 512, 512}, 512}; 

const Animation anim_walk_lft = {{anim_walk_lft_1, anim_walk_lft_2, anim_walk_lft_3, anim_walk_lft_4}, {200, 200, 200, 200}, 4, true};
const Animation anim_walk_rht = {{anim_walk_lft_4, anim_walk_lft_3, anim_walk_lft_2, anim_walk_lft_1}, {200, 200, 200, 200}, 4, true};

  // pace f/b
const RobotState anim_pace_fwd_1 = {{712, 712, 512}, {512, 512, 512}, {712, 512, 512}, {512, 712, 512}, 512};
const RobotState anim_pace_fwd_2 = {{712, 512, 512}, {512, 512, 512}, {712, 512, 512}, {512, 512, 512}, 512}; 
const RobotState anim_pace_fwd_3 = {{512, 512, 512}, {712, 712, 512}, {512, 712, 512}, {712, 512, 512}, 512}; 
const RobotState anim_pace_fwd_4 = {{512, 512, 512}, {712, 512, 512}, {512, 512, 512}, {712, 512, 512}, 512}; 

const Animation anim_pace_fwd = {{anim_pace_fwd_1, anim_pace_fwd_2, anim_pace_fwd_3, anim_pace_fwd_4}, {100, 100, 100, 100}, 4, true};
const Animation anim_pace_bwd = {{anim_pace_fwd_4, anim_pace_fwd_3, anim_pace_fwd_2, anim_pace_fwd_1}, {100, 100, 100, 100}, 4, true};

  // pace f/b
const RobotState anim_pace_lft_1 = {{312, 712, 512}, {512, 512, 512}, {312, 512, 512}, {512, 712, 512}, 512};
const RobotState anim_pace_lft_2 = {{312, 512, 512}, {512, 512, 512}, {312, 512, 512}, {512, 512, 512}, 512}; 
const RobotState anim_pace_lft_3 = {{512, 512, 512}, {312, 712, 512}, {512, 712, 512}, {312, 512, 512}, 512}; 
const RobotState anim_pace_lft_4 = {{512, 512, 512}, {312, 512, 512}, {512, 512, 512}, {312, 512, 512}, 512}; 

const Animation anim_pace_lft = {{anim_pace_lft_1, anim_pace_lft_2, anim_pace_lft_3, anim_pace_lft_4}, {100, 100, 100, 100}, 4, true};
const Animation anim_pace_rht = {{anim_pace_lft_4, anim_pace_lft_3, anim_pace_lft_2, anim_pace_lft_1}, {100, 100, 100, 100}, 4, true};

/* 
 *  Doesn't work with tetra
  // trot
RobotState anim_trot_1 = {{712, 712, 512}, {512, 512, 512}, {512, 712, 512}, {712, 512, 512}, 512};
RobotState anim_trot_2 = {{712, 512, 512}, {512, 512, 512}, {512, 512, 512}, {712, 512, 512}, 512}; 
RobotState anim_trot_3 = {{512, 512, 512}, {712, 712, 512}, {712, 512, 512}, {512, 712, 512}, 512}; 
RobotState anim_trot_4 = {{512, 512, 512}, {712, 512, 512}, {712, 512, 512}, {512, 512, 512}, 512}; 

Animation anim_trot = {{anim_trot_1, anim_trot_2, anim_trot_3, anim_trot_4}, {50, 50, 50, 50}, 4, true};
*/

// Play Animations

Animation anim_active = {};
int current_frame = 0;
int next_frame_time = 0;

void init_animation(Animation new_animation){
  anim_active = new_animation;
  current_frame = 0;
  next_frame_time = 0;
}

void play_animation(){
  if(anim_active.is_looping && current_frame >= anim_active.number_of_frames){
      current_frame = 0; 
    }
  if(current_frame < anim_active.number_of_frames){
    
    long current_time = millis();
    if(current_time >= next_frame_time){
      cfg_active.LeftFront  = anim_active.frames[current_frame].LeftFront;
      cfg_active.RightFront = anim_active.frames[current_frame].RightFront;
      cfg_active.LeftBack   = anim_active.frames[current_frame].LeftBack;
      cfg_active.RightBack  = anim_active.frames[current_frame].RightBack;
      
      next_frame_time = current_time + anim_active.lengths[current_frame];
      current_frame ++;
    }
    
  }
  
}


// ------------------------------- SETUP  ------------------------------- // 

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  irrecv.enableIRIn();
  irrecv.blink13(true);

  Serial.println("Started");


  cfg_active = cfg_start;
 
}


// ----------------------------- MAIN LOOP ----------------------------- // 


String command = "";
long state = SLEEP_LOOP;

#define DIRECTION_FORWARD 0
#define DIRECTION_BACKWARD 1
#define DIRECTION_LEFT 2
#define DIRECTION_RIGHT 3
int dir = DIRECTION_FORWARD;
void loop() {
  //Serial.println(state);
  // STATE MACHINE
  // ----  SLEEP  ---- //
  if(state == SLEEP){
    state = SLEEP_LOOP;
    
    init_animation(anim_sleep);
    init_melody(mel_sleep);
    Serial.println("STATUS - SLEEP");
  }
  else if(state == SLEEP_LOOP){
    // EXIT CASES
    if(ir_value == IR_POWER){
      state = AWAKE;
    }
  }
  
  // ----  AWAKE  ---- //
  else if(state == AWAKE){
    init_animation(anim_awake);
    state = AWAKE_LOOP;
    init_melody(mel_awake);
    Serial.println("STATUS - AWAKE");
  }
  else if(state == AWAKE_LOOP){
    // EXIT CASES
    if(ir_value == IR_POWER){
      state = SLEEP;
    }
    else if(ir_value == IR_PLAY){
      state = WALK;
    }
    else if(ir_value == IR_TWO){
      dir = DIRECTION_FORWARD;
    }
    else if(ir_value == IR_EIGHT){
      dir = DIRECTION_BACKWARD;
    }
    else if(ir_value == IR_FOUR){
      dir = DIRECTION_LEFT;
    }
    else if(ir_value == IR_SIX){
      dir = DIRECTION_RIGHT;
    }
  }
  
  // ----  WALK  ---- //
  else if(state == WALK){
    
    state = WALK_LOOP;
    
    Serial.println("STATUS - WALKING");
  
    switch(dir){
      case DIRECTION_FORWARD:
        init_animation(anim_walk_fwd);
        break;
      case DIRECTION_BACKWARD:
        init_animation(anim_walk_bwd);
        break;
      case DIRECTION_LEFT:
        init_animation(anim_walk_lft);
        break;
      case DIRECTION_RIGHT:
        init_animation(anim_walk_rht);
        break;
    }
  }
  else if(state == WALK_LOOP){
    
    // IR FUNCTIONS + EXIT CASES
    if(ir_value == IR_FFWD){
      init_melody(mel_ping);
      state = PACE;
    }
    else if(ir_value == IR_RWD){
      init_melody(mel_error);
    }
    else if(ir_value == IR_POWER){
      state = SLEEP;
    }
    else if(ir_value == IR_PLAY){
      init_animation(anim_stand);
      Serial.println("STATUS - STANDING");
      state = AWAKE_LOOP;
    }
    else if(ir_value == IR_TWO){
      dir = DIRECTION_FORWARD;
      state = WALK;
    }
    else if(ir_value == IR_EIGHT){
      dir = DIRECTION_BACKWARD;
      state = WALK;
    }
    else if(ir_value == IR_FOUR){
      dir = DIRECTION_LEFT;
      state = WALK;
    }
    else if(ir_value == IR_SIX){
      dir = DIRECTION_RIGHT;
      state = WALK;
    }
  }
  
  // ----  PACE  ---- //
  else if(state == PACE){
    switch(dir){
      case DIRECTION_FORWARD:
        init_animation(anim_pace_fwd);
        break;
      case DIRECTION_BACKWARD:
        init_animation(anim_pace_bwd);
        break;
      case DIRECTION_LEFT:
        init_animation(anim_pace_lft);
        break;
      case DIRECTION_RIGHT:
        init_animation(anim_pace_rht);
        break;
    }
    
    state = PACE_LOOP;
    Serial.println("STATUS - PACING");
  }
  else if(state == PACE_LOOP){
    // IR FUNCTIONS + EXIT CASES
    if(ir_value == IR_RWD){
      init_melody(mel_ping);
      state = WALK;
    }
    else if(ir_value == IR_FFWD){
      init_melody(mel_error);
    }
    else if(ir_value == IR_POWER){
      state = SLEEP;
    }
    else if(ir_value == IR_PLAY){
      Serial.println("STATUS - STANDING");
      init_animation(anim_stand);
      state = AWAKE_LOOP;
    }
    else if(ir_value == IR_TWO){
      dir = DIRECTION_FORWARD;
      state = PACE;
    }
    else if(ir_value == IR_EIGHT){
      dir = DIRECTION_BACKWARD;
      state = PACE;
    }
    else if(ir_value == IR_FOUR){
      dir = DIRECTION_LEFT;
      state = PACE;
    }
    else if(ir_value == IR_SIX){
      dir = DIRECTION_RIGHT;
      state = PACE;
    }
  }
  
  play_animation();
  write_servos();
  readIR();
  //print_robot_state();
  play_melody();
  delay(20);
}

//https://arduino.stackexchange.com/questions/1013/how-do-i-split-an-incoming-string

// ----------------------------- UTILITY ----------------------------- //

void print_robot_state(){
  Serial.println("Tetra State \n-----------------------------------");
  
  Serial.print("Left Front: ");
  Serial.print("Roator - " + String(cfg_active.LeftFront.Rotator) + " ");
  Serial.print("Lift - " + String(cfg_active.LeftFront.Lift) + " ");
  Serial.println("Kick - " + String(cfg_active.LeftFront.Kick)+ " ");

  Serial.print("Right Front: ");
  Serial.print("Roator - " + String(cfg_active.RightFront.Rotator) + " ");
  Serial.print("Lift - " + String(cfg_active.RightFront.Lift)+ " ");
  Serial.println("Kick - " + String(cfg_active.RightFront.Kick)+ " ");
  
  Serial.print("Left Back: ");
  Serial.print("Roator - " + String(cfg_active.LeftBack.Rotator) + " ");
  Serial.print("Lift - " + String(cfg_active.LeftBack.Lift)+ " ");
  Serial.println("Kick - " + String(cfg_active.LeftBack.Kick) + " ");

  Serial.print("Left Back: ");
  Serial.print("Roator - " + String(cfg_active.RightBack.Rotator) + " ");
  Serial.print("Lift - " + String(cfg_active.RightBack.Lift) + " ");
  Serial.println("Kick - " + String(cfg_active.RightBack.Kick) + " ");
}

// ----------------------------- SERVO CONTROL ----------------------------- // 

void set_leg(String leg, int Rotator, int Lift, int Kick){
  if(leg == "LF") {
      cfg_active.LeftFront = {Rotator, Lift, Kick};
  }
  else if(leg == "LB") {
      cfg_active.LeftBack = {Rotator, Lift, Kick};
  }
  else if(leg == "RF") {
      cfg_active.RightFront = {Rotator, Lift, Kick};
  }
  else if(leg == "RB") {
      cfg_active.RightFront = {Rotator, Lift, Kick};
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
  pwm.setPWM(ID_Rotator_LF, 0, map(cfg_active.LeftFront.Rotator + fine_offset.LeftFront.Rotator, 0, 1024,   lower_limit.LeftFront.Rotator,  upper_limit.LeftFront.Rotator));
  pwm.setPWM(ID_Lift_LF,    0, map(cfg_active.LeftFront.Lift    + fine_offset.LeftFront.Lift, 0, 1024,      lower_limit.LeftFront.Lift,     upper_limit.LeftFront.Lift   ));
  pwm.setPWM(ID_Kick_LF,    0, map(cfg_active.LeftFront.Kick    + fine_offset.LeftFront.Kick, 0, 1024,      lower_limit.LeftFront.Kick,     upper_limit.LeftFront.Kick   ));

  pwm.setPWM(ID_Rotator_LB, 0, map(cfg_active.LeftBack.Rotator + fine_offset.LeftBack.Rotator, 0, 1024,     lower_limit.LeftBack.Rotator,   upper_limit.LeftBack.Rotator));
  pwm.setPWM(ID_Lift_LB,    0, map(cfg_active.LeftBack.Lift    + fine_offset.LeftBack.Lift, 0, 1024,        lower_limit.LeftBack.Lift,      upper_limit.LeftBack.Lift   ));
  pwm.setPWM(ID_Kick_LB,    0, map(cfg_active.LeftBack.Kick    + fine_offset.LeftBack.Kick, 0, 1024,        lower_limit.LeftBack.Kick,      upper_limit.LeftBack.Kick   ));

  pwm.setPWM(ID_Rotator_RF, 0, map(cfg_active.RightFront.Rotator + fine_offset.RightFront.Rotator, 0, 1024, lower_limit.RightFront.Rotator, upper_limit.RightFront.Rotator));
  pwm.setPWM(ID_Lift_RF,    0, map(cfg_active.RightFront.Lift    + fine_offset.RightFront.Lift, 0, 1024,    lower_limit.RightFront.Lift,    upper_limit.RightFront.Lift   ));
  pwm.setPWM(ID_Kick_RF,    0, map(cfg_active.RightFront.Kick    + fine_offset.RightFront.Kick, 0, 1024,    lower_limit.RightFront.Kick,    upper_limit.RightFront.Kick   ));

  pwm.setPWM(ID_Rotator_RB, 0, map(cfg_active.RightBack.Rotator + fine_offset.RightBack.Rotator, 0, 1024,   lower_limit.RightBack.Rotator,  upper_limit.RightBack.Rotator));
  pwm.setPWM(ID_Lift_RB,    0, map(cfg_active.RightBack.Lift    + fine_offset.RightBack.Lift, 0, 1024,      lower_limit.RightBack.Lift,     upper_limit.RightBack.Lift  ));
  pwm.setPWM(ID_Kick_RB,    0, map(cfg_active.RightBack.Kick    + fine_offset.RightBack.Kick, 0, 1024,      lower_limit.RightBack.Kick,     upper_limit.RightBack.Kick  ));

  pwm.setPWM(ID_Neck,       0, map(cfg_active.Neck + fine_offset.Neck, 0, 1024, lower_limit.Neck, upper_limit.Neck  ));

}
