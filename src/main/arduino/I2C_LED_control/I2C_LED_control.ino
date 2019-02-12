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
String currentRoutine;
TBlendType currentBlending;
uint8_t currentBrightness = BRIGHTNESS;

const TProgmemPalette16 Green_p PROGMEM = { CRGB::Green};
const CRGB GREEN = CRGB::Green;
const TProgmemPalette16 Blue_p PROGMEM = { CRGB::Blue};
const CRGB BLUE = CRGB::Blue;
const TProgmemPalette16 Yellow_p PROGMEM = { CRGB::Yellow};
const CRGB YELLOW = CRGB::Yellow;
const TProgmemPalette16 Red_p PROGMEM = { CRGB::Red};
const CRGB RED = CRGB::Red;

const TProgmemPalette16 greenBlue_p PROGMEM = {CRGB::Yellow, CRGB::Blue};
// Avaliable palettes include RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.

boolean lit = true;
static uint8_t startIndex = 0;

boolean raising = false;
int lastExtream;
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
  currentBlending = LINEARBLEND;
  currentRoutine = "snake";
  
  Wire.begin(8);// address #8
  Wire.onReceive(receiveEvent);
  
}

void loop() {
  //delay(100);
  FastLED.setBrightness(currentBrightness);
  
  if(lit){
    if(currentRoutine == "snake"){
      startIndex = startIndex + 1;//Motion Speed
      snakeFromPalette(startIndex);
    } 
    else if(currentRoutine == "solid"){
      fill_solid(currentPalette, NUM_LEDS, currentColor);
    }
    else if(currentRoutine == "fade"){
      fill_solid(currentPalette, NUM_LEDS, currentColor);
      //fadeFromPalette();
    }

  }  
    FastLED.show();
    FastLED.delay(1000/UPDATES_PER_SECOND);  
}

// this function is registered as an event, see setup() reeee black box code
void receiveEvent(int howMany) {
  char set;
  char data;
  
  while (1 < Wire.available()){
    set = Wire.read() - '0';
    //Serial.print(c);
  }
  data = Wire.read() - '0';
  //Serial.println(c);
  if(set == 0)
    setColor(data);
  else if (set == 1)  
    setRoutine(data);
  else if (set == 2)
    lit = false;
}

void setColor(int color){
  
  if(lit && color != colorLast){
    //Serial.priint("Setting color to ");
    //Serial.println(data);
    switch(color){
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
        currentColor = GREEN;
        break;  
    }  
  }    
  colorLast = color;
  
}

void setRoutine(int routine){
  if(lit && routine != routineLast){
   //Serial.priint("Setting routine to ");
   //Serial.println(data);
    switch(routine){
       case 0:
         currentRoutine = "snake";
         break;
       case 1:
         currentRoutine = "solid";
         break;
       case 2:
         currentRoutine = "fade";
         break; 
    }   
  }   
  routineLast = routine;
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

  if(BRIGHTNESS == MAX){
    raising = false;
  }
  else if(BRIGHTNESS == MIN){
    raising = true;
  }
  
  if(raising){
    currentBrightness++;
  }else
    currentBrightness--;
  
}

