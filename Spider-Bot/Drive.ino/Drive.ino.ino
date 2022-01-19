#include <math.h>
#include <Servo.h>

#include "pitches.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

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

const int servo_min = 90;
const int servo_max = 470;
const int servo_mid = 280;

const int thigh_length = 1;
const int calf_length = 1;

const int kick_offset = 45;
const int lift_offset = 60;

const int Rotator_LF = 0;
const int Lift_LF = 1;
const int Kick_LF = 2;

const int Rotator_RF = 4;
const int Lift_RF = 5;
const int Kick_RF = 6;

const int Rotator_LB = 8;
const int Lift_LB = 9;
const int Kick_LB = 10;

const int Rotator_RB = 12;
const int Lift_RB = 13;
const int Kick_RB = 14;

typedef struct  {
  int Rotator;
  int Lift;
  int Kick;
} LegPosition;

typedef struct  {
  LegPosition LeftFront;
  LegPosition RightFront;
  LegPosition LeftBack;
  LegPosition RightBack;
} RobotState;

typedef struct {
  RobotState frames [4];
  int frame_delay;
  bool is_looping;
}Animation;

typedef struct {
  float x;
  float y;
  float z;
} Vector3;

typedef struct {
  float d;
  float z;
} Vector2;




// Parameters
const RobotState lower_limit = {{470, 90, 120}, {90, 470, 400}, {90, 470, 380}, {470, 90, 120},};
const RobotState upper_limit = {{90, 470, 400}, {470, 90, 120}, {470, 90, 140}, {90, 470, 400},};

const RobotState fine_offset = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Configurations

RobotState spider = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};


RobotState cfg_start = {{512, 512, 512}, {312, 712, 512}, {712, 712, 512}, {512, 512, 512}};
RobotState cfg_big = {{512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}};

RobotState anim_walk_fwd_1 = {{512, 512, 512}, {312, 712, 512}, {712, 712, 512}, {512, 512, 512}};
RobotState anim_walk_fwd_2 = {{512, 512, 512}, {312, 512, 512}, {712, 512, 512}, {512, 512, 512}}; 
RobotState anim_walk_fwd_3 = {{512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}}; 

RobotState anim_walk_fwd_4 = {{312, 712, 512}, {512, 512, 512}, {512, 512, 512}, {712, 712, 512}};
RobotState anim_walk_fwd_5 = {{312, 512, 512}, {512, 512, 512}, {512, 512, 512}, {712, 512, 512}}; 
RobotState anim_walk_fwd_6 = {{512, 512, 512}, {512, 512, 512}, {512, 512, 512}, {512, 512, 512}}; 

Animation anim_walk_fwd = {{anim_walk_fwd_1, anim_walk_fwd_2, anim_walk_fwd_4, anim_walk_fwd_5}, 100, true};

/*
// Speakers

const int Speaker_LF = 8;
const int Speaker_RF = 9;
const int Speaker_LB = 10;
const int Speaker_RB = 11;

typedef struct {
  int root;
  int first;
  int second;
  int third;
  int duration;
} Chord;

Chord start_up_melody[4] = {{NOTE_C2, NOTE_F2, NOTE_G2, 0, 4}, {NOTE_A3, NOTE_C3, NOTE_D3, 0, 4}, {NOTE_F2, NOTE_A3, NOTE_C3, 0, 4}, {NOTE_G2, NOTE_B3, NOTE_D3, 0, 4}};
const float note_delay_time = 1.3;

int next_chord_time = 0;
Chord active_melody[4] = {{}, {}, {}, {}};
int current_node = 0;



void initalize_melody(Chord new_melody [4]){
  for(int i = 0; i != 4; i++){
    active_melody[i].root   = new_melody[i].root;
    active_melody[i].first  = new_melody[i].first;
    active_melody[i].second = new_melody[i].second;
    active_melody[i].third  = new_melody[i].third;
    active_melody[i].duration  = new_melody[i].duration;
  }
  current_node = 0;
  next_chord_time = 0;
}

void play_melody(){
  // Some code from toneMelody example
  if(current_node != 4){
  int current_time = millis();
  if(current_time >= next_chord_time){
    int note_duration = 4000 /  active_melody[current_node].duration;
    next_chord_time = note_duration + current_time;
    Serial.println("Playing Note: " + String(current_node) +" uintil:" + String(active_melody[current_node].duration) + " @" + String(current_time)); 
     
    
    
    if(active_melody[current_node].root != 0){
      tone(Speaker_LF, active_melody[current_node].root,   note_duration);
      Serial.println(active_melody[current_node].root);
    }
  
    if(active_melody[current_node].first != 0){
      tone(Speaker_RF, active_melody[current_node].first,  note_duration);
    } 
    
    if(active_melody[current_node].second != 0){
      tone(Speaker_LB, active_melody[current_node].second, note_duration);
    } 
    
    if(active_melody[current_node].third != 0){
      tone(Speaker_RB, active_melody[current_node].third,  note_duration);
    } 
    
    current_node ++;
    noTone(Speaker_LF);
    noTone(Speaker_RF);
    noTone(Speaker_LB);
    noTone(Speaker_RB);
  }
  }

 
}
*/




#include <IRremote.h>
#include "buttons.h"

const int RECV_PIN = 4;
IRrecv irrecv(RECV_PIN);
decode_results ir_results;

long current_ir_value = 0;
long last_ir_value = 0;

long ir_value = 0;

void readIR(){
  irrecv.decode(&ir_results);
  irrecv.resume();
  //Serial.println(String(ir_results.value) + " " + String(current_ir_value) + " " + String(last_ir_value));

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

// ----------------------------- ANIMATION ----------------------------- \\ 

Animation current_animation = {};
int current_frame_index = 0;
int next_frame_time = 0;

void set_animation(Animation new_animation){
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
      spider = current_animation.frames[current_frame_index];

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
  
  set_animation(anim_walk_fwd);
}


// ----------------------------- MAIN LOOP ----------------------------- \\ 

String command = "";
void loop() {
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
     
      set_leg(leg, rotate, lift, kick);
    }
    else if(command.startsWith("set")){
      int id = Serial.readStringUntil(' ').toInt();
      int freq = Serial.readStringUntil(' ').toInt();

      Serial.println("Moving " + String(id) + " to " + String(freq));
      write_servo(id, freq);
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
  Serial.println(ir_value);
  switch(ir_value){
    case IR_POWER:
      Serial.println("IR_POWER");
      break;
      
    case IR_MODE:
      Serial.println("IR_MODE");
      break;
      
    case IR_MUTE:
      Serial.println("IR_MUTE");
      break;
      
    case IR_PLAY:
      Serial.println("IR_PLAY");
      break;
      
    case IR_RWD:
      Serial.println("IR_RWD");
      break;
      
    case IR_FFWD:
      Serial.println("IR_FFWD");
      break;
      
    case IR_EQ:
      Serial.println("IR_EQ");
      break;
      
    case IR_MINUS:
      Serial.println("IR_MINUS");
      break;
      
    case IR_PLUS:
      Serial.println("IR_PLUS");
      break;
      
    case IR_ZERO:
      Serial.println("IR_ZERO");
      break;
      
    case IR_TWIST:
      Serial.println("IR_TWIST");
      break;
      
    case IR_USD:
      Serial.println("IR_USD");
      break;
      
    case IR_ONE:
      Serial.println("IR_ONE");
      break;
      
    case IR_TWO:
      Serial.println("IR_TWO");
      break;
      
    case IR_THREE:
      Serial.println("IR_THREE");
      break;
      
    case IR_FOUR:
      Serial.println("IR_FOUR");
      break;
      
    case IR_FIVE:
      Serial.println("IR_FIVE");
      break;
      
    case IR_SIX:
      Serial.println("IR_SIX");
      break;
      
    case IR_SEVEN:
      Serial.println("IR_SEVEN");
      break;
      
    case IR_EIGHT:
      Serial.println("IR_EIGHT");
      break;
      
    case IR_NINE:
      Serial.println("IR_NINE");
      break;

    default:
      if(ir_value != 0){
        
      Serial.println("BAD DATA");
      }
      break;
  }

  play_animation();
  write_servos();
  readIR();
  //play_melody();
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

// ----------------------- INVERSE KINEMATICS ------------------------ \\ 

double dist3(double x, double y, double z){
  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

double dist3(Vector3 t){
  return sqrt(pow(t.x, 2) + pow(t.y, 2) + pow(t.z, 2));
}

double dist2(double x, double y){
  return sqrt(pow(x, 2) + pow(y, 2));
}

double dist2(Vector2 t){
  return sqrt(pow(t.d, 2) + pow(t.z, 2));
}


double cos_law(double a, double b, double c){
  // find angle opposite to a
  return degrees(acos((a*a - b*b - c*c) / (2*a*b)));
}

double * calculate_angles(Vector3 t){
  
  double rotator_angle = degrees(atan((abs(t.y))/(abs(t.x))));
  
  Vector2 ref = (Vector2){dist2(t.x, t.y), t.z};
  
  double delta_d = 1;
  double hypotenuse = dist3(t.x, t.y, t.z);
      
  // calculate lift angle
  double lift_angle = degrees(atan(ref.z / ref.d) + cos_law(calf_length, thigh_length, hypotenuse)) + lift_offset;
  
  // calculate kick angle
  double kick_angle = 180 - cos_law(hypotenuse, thigh_length, calf_length) + kick_offset;
  
  double angles [3] = {rotator_angle, lift_angle, kick_angle};
  return angles;
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

void TEMP_set_config(){
  pwm.setPWM(Rotator_LF, 0, spider.LeftFront.Rotator);
  pwm.setPWM(Lift_LF,    0, spider.LeftFront.Lift);
  pwm.setPWM(Kick_LF,    0, spider.LeftFront.Kick);

  pwm.setPWM(Rotator_LB, 0, spider.LeftBack.Rotator);
  pwm.setPWM(Lift_LB,    0, spider.LeftBack.Lift);
  pwm.setPWM(Kick_LB,    0, spider.LeftBack.Kick);

  pwm.setPWM(Rotator_RF, 0, spider.RightFront.Rotator);
  pwm.setPWM(Lift_RF,    0, spider.RightFront.Lift);
  pwm.setPWM(Kick_RF,    0, spider.RightFront.Kick);

  pwm.setPWM(Rotator_RB, 0, spider.RightBack.Rotator);
  pwm.setPWM(Lift_RB,    0, spider.RightBack.Lift);
  pwm.setPWM(Kick_RB,    0, spider.RightBack.Kick);

  Serial.println("wrote state");
  print_robot_state();

}

void write_servos(){
  pwm.setPWM(Rotator_LF, 0, map(spider.LeftFront.Rotator + fine_offset.LeftFront.Rotator, 0, 1024,   lower_limit.LeftFront.Rotator,  upper_limit.LeftFront.Rotator));
  pwm.setPWM(Lift_LF,    0, map(spider.LeftFront.Lift    + fine_offset.LeftFront.Lift, 0, 1024,      lower_limit.LeftFront.Lift,     upper_limit.LeftFront.Lift   ));
  pwm.setPWM(Kick_LF,    0, map(spider.LeftFront.Kick    + fine_offset.LeftFront.Kick, 0, 1024,      lower_limit.LeftFront.Kick,     upper_limit.LeftFront.Kick   ));

  pwm.setPWM(Rotator_LB, 0, map(spider.LeftBack.Rotator + fine_offset.LeftBack.Rotator, 0, 1024,     lower_limit.LeftBack.Rotator,   upper_limit.LeftBack.Rotator));
  pwm.setPWM(Lift_LB,    0, map(spider.LeftBack.Lift    + fine_offset.LeftBack.Lift, 0, 1024,        lower_limit.LeftBack.Lift,      upper_limit.LeftBack.Lift   ));
  pwm.setPWM(Kick_LB,    0, map(spider.LeftBack.Kick    + fine_offset.LeftBack.Kick, 0, 1024,        lower_limit.LeftBack.Kick,      upper_limit.LeftBack.Kick   ));

  pwm.setPWM(Rotator_RF, 0, map(spider.RightFront.Rotator + fine_offset.RightFront.Rotator, 0, 1024, lower_limit.RightFront.Rotator, upper_limit.RightFront.Rotator));
  pwm.setPWM(Lift_RF,    0, map(spider.RightFront.Lift    + fine_offset.RightFront.Lift, 0, 1024,    lower_limit.RightFront.Lift,    upper_limit.RightFront.Lift   ));
  pwm.setPWM(Kick_RF,    0, map(spider.RightFront.Kick    + fine_offset.RightFront.Kick, 0, 1024,    lower_limit.RightFront.Kick,    upper_limit.RightFront.Kick   ));

  pwm.setPWM(Rotator_RB, 0, map(spider.RightBack.Rotator + fine_offset.RightBack.Rotator, 0, 1024,   lower_limit.RightBack.Rotator,  upper_limit.RightBack.Rotator));
  pwm.setPWM(Lift_RB,    0, map(spider.RightBack.Lift    + fine_offset.RightBack.Lift, 0, 1024,      lower_limit.RightBack.Lift,     upper_limit.RightBack.Lift  ));
  pwm.setPWM(Kick_RB,    0, map(spider.RightBack.Kick    + fine_offset.RightBack.Kick, 0, 1024,      lower_limit.RightBack.Kick,     upper_limit.RightBack.Kick  ));

  

}
