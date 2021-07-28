#include "Arduino.h" 
#include "Encoder.h"




#define D6 PD6
#define D7 PD7
#define D8 PB0


const int RE_BUT = 6; // rotary encoder button  // NOTE:  library does pinmode pullup
const int RE_A = 7; // rotary encoder motion
const int RE_B = 8; // rotary encoder motion



Encoder encoder(RE_A, RE_B, RE_BUT); // set up rotary encoder

  /*     ----------    S E T U P     --------- */


  void setup() {

    Serial.begin(9600);
    Serial.println("re test online");

    // rotary encoder
    EncoderInterrupt.begin( &encoder );

  }

  /*     ----------    L O O P     --------- */



  void loop() {

    // do this, even if we ignore, to prevent buildup

    // read the debounced value of the encoder button
    bool pb = encoder.button();
    // get the encoder variation since our last check, it can be positive or negative, or zero if the encoder didn't move
    // only call this once per loop cicle, or at any time you want to know any incremental change
    int delta = encoder.delta();
   
    if (delta != 0) {
      Serial.println("delta = "+String(delta));
    }
    if (pb) {
      Serial.println("RE button press");
    }

  }
