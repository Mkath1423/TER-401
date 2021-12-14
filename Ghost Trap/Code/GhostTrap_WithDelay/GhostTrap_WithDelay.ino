#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// PINS:
// ------------------------------------
// LED Data Pin: Digital Pin 6
// Button Pin: Digital Pin 8
// Door Servos: Digital Pins 9 and 10

#define LED_PIN     6
#define LED_COUNT  128
#define BRIGHTNESS 50

#define INPUT_PIN 8

#define RED_PIN 5
#define GREEN_PIN 3

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

Servo door_l;
Servo door_r;

unsigned int last_time;

void setup() {

  // Input Pin / Button
  pinMode(INPUT_PIN, INPUT_PULLUP);

  // LEDs
  strip.begin();           
  strip.show();           
  strip.setBrightness(50); 

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
                           
  // Servos
  door_l.attach(9);
  door_r.attach(10);

  door_l.write(90);
  door_r.write(90);

  last_time = millis();
  
  Serial.begin(9600);
}

// Track state of door and button for polling
bool play_animation  = false;
bool door_open = false;
bool last_button_state = false;

int ghost_trapped_counter = 0;
bool ghost_trapped = false;

unsigned int current_time = 0;
unsigned int delta_time = 0;

int toggle_timer = 0;
int toggle_timer_max = 1000; 
bool count_time = false;

void loop() {
  // for timing
  current_time = millis();
  delta_time = current_time - last_time;
  last_time = current_time;

  if(count_time){
    toggle_timer += delta_time;
  }
  
  // If the button is clicked down 
  // Toggle the door state
  bool input_value = !digitalRead(INPUT_PIN);
  
  if(input_value && !last_button_state){
    count_time = ! count_time;
    door_open = ! door_open;
  }
  
  last_button_state = input_value;

  if(toggle_timer > toggle_timer_max){
    toggle_timer = 0;
    count_time = false;

    play_animation = ! play_animation;
    ghost_trapped_counter += 1;
  }

  if(door_open){
    // Set Servos to open
    door_l.write(30);
    door_r.write(120);
  }
  else{
    // Set Setvos to Close
    door_l.write(120);
    door_r.write(30);
  }
  
  if(play_animation){
    // Play animation
    ColoredLines();
  }
  else{
    // Clear the LEDs
    strip.clear();
    strip.show();
  }

  // When the door is opened and closed 
  // toggle the light between red and green
  if(ghost_trapped_counter == 2){
    Serial.println("Counter Reached 2");
    ghost_trapped_counter = 0;
    ghost_trapped = ! ghost_trapped;
  }

  if(!ghost_trapped){
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, HIGH);
  }
  else{
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(GREEN_PIN, LOW);
  }

  //Serial.println(String(ghost_trapped) + " " + String(ghost_trapped_counter));
  delay(10);
}

// Animation for random colors 
const uint8_t line_starters[] = {0, 1, 2, 3, 4, 5, 6, 7, 64, 65, 66, 67, 68, 69, 70, 71};

void ColoredLines(){
  for(int i = 0; i!=16; i++){
    uint32_t color = Adafruit_NeoPixel::ColorHSV(random(65536), 150, 255);
    
    setLineColor(line_starters[i], color);
  }
  
  strip.show();
}

void setLineColor(int lineNum, uint32_t color){
  for(int i =0; i!=8; i++){
    strip.setPixelColor(lineNum+(8*i), color);
  }
}

// Animation for cycling through hues
int hue = 0;

void HueCycle(){
  uint32_t color = Adafruit_NeoPixel::ColorHSV(hue, 200, 255);
  
  for(int i = 0; i<strip.numPixels(); i++){
    strip.setPixelColor(i, color);
  }
  
  strip.show();
  hue = hue + 65536/100;
}

void fill(uint32_t color){
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
  }
  strip.show();  
}
