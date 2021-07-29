#include "Encoder.h"

/*
 *                ____
 *  RE_A (7)   - |    |_  RE_BUT (6)
 *  GND        - |    |_  GND
 *  RE_B (8)   - |____|
 * 
 * 
 * 
 */





const int RE_BUT = 6; // rotary encoder button  // NOTE:  library does pinmode pullup
const int RE_A = 7; // rotary encoder motion
const int RE_B = 8; // rotary encoder motion


int foo = 128;
int prev_foo = 128;


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
   
    if (pb) {
      Serial.println("RE button press");
    }
    
    
    
    // get the encoder variation since our last check, it can be positive or negative, or zero if the encoder didn't move
    // only call this once per loop cicle, or at any time you want to know any incremental change
    int delta = encoder.delta();
    //Serial.println("foo = "+String(foo));

    if (abs(delta) <= 1 || abs(delta) >= 255) {
      Serial.println("small delta "+String(delta));
      return;
    } else {
      Serial.print("====== ");
    }

    
    Serial.println("delta = "+String(delta));
    int new_foo = foo - delta;
    if (new_foo > 256) {
      foo = 256;
    } else if (new_foo < 0) {
      foo = 0;
    } else {
      foo = new_foo;
    }
    Serial.println("delta = "+String(delta) +"   foo = "+String(foo));




  }
