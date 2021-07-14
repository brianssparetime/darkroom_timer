#include "Arduino.h" 
#include <TM1637.h>
#include "Encoder.h"
#include <LiquidCrystal_I2C.h>




/*
 * TODO: 
 *   - write buzzer not to use delay()
 *   - expand menu
 *     - easier state additions
 *     - support film dev schedule
 *   - check for pin swap on RE_BUT and if hardwired pull up resistor is a problem ???
 *   - debounce buttons SLS ENS STRT ???

 */


#define DEBUG


const int MAX_TIME = 60 * 60 -1; // 1 hour
const int BZR_FREQ = 261; // hz freq for the buzzer/beeper

// --------- pins -----------

const int CLK = A3; // 4seg
const int DIO = A2; // 4seg
//arduino pin mapping 
//see https://www.electronicshub.org/wp-content/uploads/2021/01/Arduino-Nano-Pinout.jpg
#define D4 PD4
#define D5 PD5
#define D3 PD3
#define D6 PD6
#define D7 PD7
#define D8 PB0
#define D9 PB1

// enlarger and safelight switches and relays
const int ENS = D4; // enlarger switch, active high, pulldown
const int SLS = D5; // safelight switch, active high, pulldown
const int ENR = A0; // enlarger relay, active low, pullup
const int SLR = A1; // safelight relay, active low, pullup

// must be pin 2 or 3 on nano for interrupts to work
const int STRT = D3; // start button and footswitch
//const int BACK = 0; // back button

const int RE_A = D7; // rotary encoder motion
const int RE_B = D6; // rotary encoder motion
const int RE_BUT = D8; // rotary encoder button  // NOTE:  library does pinmode pullup

const int BZR = D9; // buzzer or beeper



// --------    state variables -----------

int cur_tmr_val = 12; // seconds; does not count down
int new_tmr_val = 12; // seconds, for when changing
enum en_states {
  EN_ACTIVE,
  EN_OVERRIDE_ON,
  EN_IDLE
};
en_states EN_STATE = EN_IDLE;

long unsigned timer_end = 0; // store when timer finishes in ms 
long unsigned last_disp_update = 0; // used for refreshing 47seg 

enum f_states {
  F_SELECT,
  F_TIMER_EDIT
};
f_states F_STATE = F_SELECT;


bool prev_RE_button = false;
bool prev_SLS = false;
bool prev_ENS = false;

volatile int STRT_state = 0; // for button interrupt reads




TM1637 tm(CLK, DIO); // set up 47seg
Encoder encoder(RE_A, RE_B, RE_BUT); // set up rotary encoder
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display





// display a value given in seconds as mm:ss on the segment display
void displayTimeSeg(int seconds) {
  int minutes = seconds / 60;
  seconds = seconds % 60;

  tm.point(1);
  tm.display(3, seconds % 10);
  tm.display(2, seconds / 10 % 10);
  tm.display(1, minutes % 10);
  tm.display(0, minutes / 10 % 10);
}


void enlargerOff() {
    digitalWrite(ENR, HIGH); // turn off enlarger
    delay(500); // TODO: consider the wisdom of this decision
    digitalWrite(SLR, LOW); // turn on safelight
    lcd.backlight();
    EN_STATE = EN_IDLE;
    displayTimeSeg(cur_tmr_val);
    buzz();
}


void enlargerOn() {
    buzz();
    digitalWrite(SLR, HIGH); // turn off safelight
    lcd.noBacklight();
    delay(500); // TODO: consider the wisdom of this decision
    digitalWrite(ENR, LOW); // turn on enlarger
    EN_STATE = EN_ACTIVE;
    displayTimeSeg(cur_tmr_val);   
}


void updDispTimerEdit() {
    int s = cur_tmr_val % 60;
    int m = (cur_tmr_val % 3600) / 60;
    int h = cur_tmr_val / 3600;
    char sb[50];
    sprintf(sb, "> set to: %02d:%02d", m,s);
    lcd.clear();
    lcd.setCursor(0,0); // col, row
    lcd.print("  TIMER");
    lcd.setCursor(0,1); // col, row
    lcd.print(sb);  
}

void updDispFTimer() {
    lcd.clear();
    lcd.setCursor(0,0); // col, row
    lcd.print("> TIMER");
}


void start_isr() {
  
  int st = digitalRead(STRT);
  if (EN_STATE == EN_ACTIVE) {  
  
      if (st == HIGH) { // then start has just been pressed
        // if start is pressed while enlarger is active, that means
        //  we want to keep going past the timer
        EN_STATE = EN_OVERRIDE_ON;
      } else { // start has just been released
        // we want to cut off the enlarger on button release
        EN_STATE = EN_IDLE;
        enlargerOff();    
      }

  } else { // EN inactive
      if (st == HIGH) { // then start has just been pressed
        EN_STATE = EN_ACTIVE;
        enlargerOn();
      }         // don't care about release here  
  }
  
}



void buzz() {
  // TODO: rewrite to be interrupt safe using millis()
  tone(BZR, BZR_FREQ);
  delay(200);
  noTone(BZR);Serial.println("buzz");
}




  /*     ----------    S E T U P     --------- */


  void setup() {
    // put your setup code here, to run once:


    #ifdef DEBUG 
      Serial.begin(9600);
      Serial.println("Timer online");
    #endif

    // set up pins
    pinMode(ENS, INPUT); // high active
    pinMode(SLS, INPUT); // high active
    pinMode(STRT, INPUT); // high active
    attachInterrupt(digitalPinToInterrupt(STRT), start_isr, CHANGE);
    pinMode(ENR, OUTPUT); // low active
    pinMode(SLR, OUTPUT); // low active
    digitalWrite(ENR, HIGH); // technically not necessary since switches will override in loop, but for testing
    digitalWrite(SLR, LOW); 
    

    // set up other hardware

    // 4-7 segment display
    tm.init();
    //set brightness; 0-7
    tm.set(4);
    displayTimeSeg(cur_tmr_val);

    // rotary encoder
    EncoderInterrupt.begin( &encoder );

    // initialize the lcd 
    lcd.init();                      
    // Print a message to the LCD.
    lcd.backlight();
    lcd.setCursor(0,0); // col, row)
    lcd.print("  TIMER");

    // default states
    EN_STATE = EN_IDLE;
    F_STATE = F_SELECT;

  }





  /*     ----------    L O O P     --------- */



  void loop() {

    // do this even if we ignore to prevent buildup

    // read the debounced value of the encoder button
    bool pb = encoder.button();

    // get the encoder variation since our last check, it can be positive or negative, or zero if the encoder didn't move
    // only call this once per loop cicle, or at any time you want to know any incremental change
    int delta = encoder.delta();



    // if we're in the middle of making an exposure...
    if (EN_STATE == EN_ACTIVE || EN_STATE == EN_OVERRIDE_ON) {
        long unsigned now = millis(); // ms
        long unsigned timeleft = 0; // seconds
        long unsigned overtime = 0; // seconds
        if (timer_end >= now) { // timer still running
          timeleft = (timer_end - now) / 1000;
        } else {
          overtime = (now - timer_end) / 1000;
        }
        
        if (timeleft > 0) {  // if timer still running
          //if (now > 1000 + last_disp_update) {
            displayTimeSeg(timeleft);
            //last_disp_update = now;
          //}
        } else if (timeleft = 0 && EN_STATE != EN_OVERRIDE_ON) {
            // timer has finished and not in override
            EN_STATE = EN_IDLE;
            enlargerOff();
        } else if (EN_STATE == EN_OVERRIDE_ON) {
             displayTimeSeg(overtime);
        }
        return; 
   
    return;  // IGNORE REST OF LOOP
    }

    
    // enlarger inactive - pay attention to switches and RE now

    // handle manual switch states
    // remember that the switches active high but the relays are active low
    digitalWrite(ENR, !digitalRead(ENS));
    digitalWrite(SLR, !digitalRead(SLS));



    switch(F_STATE) {

      case F_SELECT: {
        // button press => enter time set
        if (pb && (pb != prev_RE_button) ) {
          // enter function settings
          F_STATE = F_TIMER_EDIT;
          updDispTimerEdit();
        }

        // rotate => select different function
        // TODO
      }

      case F_TIMER_EDIT: {
        // button press => go back to select function state
        if (pb && (pb != prev_RE_button) ) {
          // return to F_SELECT
          updDispFTimer();         
          cur_tmr_val = new_tmr_val;
          F_STATE = F_SELECT;
        }
        
        // rotate => set new timer value
        int x = new_tmr_val + delta;
        // check bounds
        if (x > 0 && x < MAX_TIME) {
          new_tmr_val = x;
          updDispTimerEdit();
        }
      }
    } // end F_STATE switch
    
    prev_RE_button = pb;

  } // end loop





  /* RESOURCES

      pinout:
      https://i.stack.imgur.com/W9Ril.png

      Power supply wiring:
      https://electronics.stackexchange.com/questions/60199/powering-arduino-nano-12volts

      RE:
      https://github.com/John-Lluch/Encoder
        https://www.instructables.com/Improved-Arduino-Rotary-Encoder-Reading/

      Button interrupts:
      https://www.allaboutcircuits.com/technical-articles/using-interrupts-on-arduino/
        rising falling
        https://forum.arduino.cc/index.php?topic=147825.0
        pull up pull down
        https://www.instructables.com/Understanding-the-Pull-up-Resistor-With-Arduino/

      TM1637 4 segment display
      https://create.arduino.cc/projecthub/ryanchan/tm1637-digit-display-arduino-quick-tutorial-ca8a93
      https://www.makerguides.com/tm1637-arduino-tutorial/

      relay
      https://randomnerdtutorials.com/guide-for-relay-module-with-arduino/

      enum switch
      https://stackoverflow.com/questions/52932529/why-does-my-switch-case-default-when-using-enums


     */
