#include "Encoder.h"

/*
 *                ____
 *  RE_A (7)   - |    |_  RE_BUT (6)
 *  GND        - |    |_  GND
 *  RE_B (8)   - |____|
 * 
 * use pullups on A and B to 5v if not using KY040 module
 * 
 */





const int RE_BUT = 6; // rotary encoder button  // NOTE:  library does pinmode pullup
const int RE_A = 7; // rotary encoder motion
const int RE_B = 8; // rotary encoder motion


int count = 0;
Encoder encoder(RE_A, RE_B, RE_BUT); // set up rotary encoder

int interval_len = 1000; //ms
long unsigned next_interval = 0;

// ISR-related
const int TRIG = 3;
volatile bool trigger = 0;
volatile int trigger_state = 0;


void start_isr() {
  trigger_state = digitalRead(TRIG);
  if (trigger_state == HIGH) {
    trigger = 1;
  }
}

  /*     ----------    S E T U P     --------- */


  void setup() {

    Serial.begin(115200);
    Serial.println("re test online");

    // rotary encoder
    EncoderInterrupt.begin( &encoder );

    pinMode(TRIG, INPUT); // high active
    attachInterrupt(digitalPinToInterrupt(TRIG), start_isr, CHANGE);

  }

  /*     ----------    L O O P     --------- */


  void loop() {

    // do this, even if we ignore, to prevent buildup

    // read the debounced value of the encoder button
    bool pb = encoder.button();
   
    if (pb) {
      Serial.println("RE button press");
    }

    if(trigger) {
      trigger = 0;
      Serial.println("trigger");
    }
    
    
    // get the encoder variation since our last check, it can be positive or negative, or zero if the encoder didn't move
    // only call this once per loop cicle, or at any time you want to know any incremental change
    int delta = encoder.delta();

    String dir = "Neutral";
    if(delta > 0) {
      dir = "Right  ";
      Serial.println(dir + ":  delta = "+String(delta) +"   foo = "+String(count));
    } else if (delta < 0) {
      dir = "Left   ";
      Serial.println(dir + ":  delta = "+String(delta) +"   foo = "+String(count));
    }
    //Serial.println(dir + ":  delta = "+String(delta) +"   foo = "+String(count));


    if (next_interval < millis()) {
      next_interval = millis() + interval_len;
      Serial.println("================================="); 
    }

  }
