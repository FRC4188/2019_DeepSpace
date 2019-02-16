#include <FastLED.h>
#include <Wire.h>

//Lights------------------------

#define LED_PIN     5
#define NUM_LEDS    120
#define BRIGHTNESS  255
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100

CRGB leds[NUM_LEDS];
CRGBPalette16 currentPalette;
CRGB currentColor;
int currentRoutine;
TBlendType currentBlending;
uint8_t currentBrightness = BRIGHTNESS;

//Colors
const TProgmemPalette16 Green_p PROGMEM = { CRGB::Green};
const CRGB GREEN = CRGB::Green;
const TProgmemPalette16 Blue_p PROGMEM = { CRGB::Blue};
const CRGB BLUE = CRGB::Blue;
const TProgmemPalette16 Yellow_p PROGMEM = { CRGB::Yellow};
const CRGB YELLOW = CRGB::Yellow;
const TProgmemPalette16 Red_p PROGMEM = { CRGB::Red};
const CRGB RED = CRGB::Red;
const TProgmemPalette16 White_p PROGMEM = { CRGB::White};
const CRGB WHITE = CRGB::White;

const TProgmemPalette16 greenBlue_p PROGMEM = {CRGB::Yellow, CRGB::Blue};
// Avaliable palettes include RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.

//Routines 
const int SNAKE = 0;
const int SOLID = 1;
const int FADE = 2;

boolean lit = true;
static uint8_t startIndex = 0;

boolean raising = false;
const int MAX = 255;
const int MIN = 0;

//Comms----------------------
int colorLast;
int routineLast;

void setup() {
  Serial.begin(9600);
  Serial.println("Serial began at 9600");
  
  delay( 3000 ); // power-up safety delay lmao saftey dum
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
    
  currentPalette = Green_p;
  currentColor = GREEN;
  currentBlending = LINEARBLEND;
  currentRoutine = SNAKE;
  
  Wire.begin(8);// address #8
  Wire.onReceive(receiveEvent);
  
}

void loop() {
  //delay(100);
  FastLED.setBrightness(currentBrightness);
  //Serial.println("Update");
  if(currentPalette == RainbowColors_p || currentPalette == RainbowStripeColors_p || currentPalette == ForestColors_p){
    currentRoutine = "snake";
  }
  
  if(lit){
    if(currentRoutine == SNAKE){
      currentBrightness = 255;
      startIndex = startIndex + 1;//Motion Speed
      snakeFromPalette(startIndex);
    } 
    if(currentRoutine == SOLID){
      currentBrightness = 255;
      for(int i = 0; i < NUM_LEDS; i++){
        leds[i] = ColorFromPalette( currentPalette, 0, currentBrightness, currentBlending);
      }
    }
    if(currentRoutine == FADE){
      fadeFromPalette();
      for(int i = 0; i < NUM_LEDS; i++){
        leds[i] = ColorFromPalette( currentPalette, 0, currentBrightness, currentBlending);
      }
    }

  }  
    FastLED.show();
    FastLED.delay(1000/UPDATES_PER_SECOND);  
}

// this function is registered as an event, see setup() reeee black box code
void receiveEvent(int howMany) {
  Serial.println("Event");
  char set;
  char data;
  
  while (1 < Wire.available()){
    set = Wire.read() - '0';
    //Serial.print("set: ");
    //Serial.println(set);
  }
  data = Wire.read() - '0';
  //Serial.print("data :");
  //Serial.println(data);
  if(set == '0' || set == 0){
    setColor(data);
  }
  else if (set == '1' || set == 1){
    setRoutine(data);
  }
  else if (set == '2' || set == 2){
    lit = false;
  }
}

void setColor(int color){
    Serial.print("Setting color to ");
    Serial.println(color);
    switch((int)color){
      case 0:
        currentPalette = Green_p;
        currentColor = GREEN;
        break;
      case 1:
        currentPalette = Blue_p;
        currentColor = BLUE;
        break;
      case 2:
        currentPalette = Yellow_p;
        currentColor = YELLOW;
        break;  
      case 3:
        currentPalette = Red_p;
        currentColor = RED;
        break;  
      case 4:
        currentPalette = RainbowColors_p;
        currentRoutine = SNAKE;
        break; 
      case 5:
        currentPalette = RainbowStripeColors_p;
        currentRoutine = SNAKE;
        break; 
      case 6:
        currentPalette = ForestColors_p;
        currentRoutine = SNAKE;
        break;

    }  
   
}

void setRoutine(int routine){
   Serial.print("Setting routine to ");
   Serial.println(routine);
    switch((int)routine){
       case 0:
         currentRoutine = SNAKE;
         break;
       case 1:
         currentRoutine = SOLID;
         break;
       case 2:
         currentRoutine = FADE;
         break; 
    }   
}

void snakeFromPalette(uint8_t colorIndex){
    currentBrightness = 255;
    uint8_t brightness = 255;
    
    for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        //leds[i] = ColorFromPalette( currentPalette, colorIndex *2, brightness, currentBlending);
        //leds[i] = ColorFromPalette( currentPalette, colorIndex *3, brightness, currentBlending);
        //leds[i] = ColorFromPalette( currentPalette, colorIndex *4, brightness, currentBlending);
        colorIndex += 1;
    }
}

void fadeFromPalette(){

//Serial.println(BRIGHTNESS

  if(currentBrightness >= MAX){
    raising = false;
  }
  else if(currentBrightness <= MIN){
    raising = true;
  }
  
  if(raising){
    currentBrightness++;
  }else{
    currentBrightness--;
  }
}

