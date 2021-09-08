#include "Arduino.h" 
#include <TM1637.h>
#include "Encoder.h"
#include <LiquidCrystal_I2C.h>




/*

  For use on Arduino Nano

 */


#define DEBUG


// --------- pins -----------

//arduino pin mapping based on Arduino Nano v3


// enlarger and safelight switches and relays
const int ENS = 4; // enlarger switch, active high, pulldown
const int SLS = 5; // safelight switch, active high, pulldown
const int ENR = A0; // enlarger relay, active low, pullup
const int SLR = A1; // safelight relay, active low, pullup
const int CLK = A3; // 4seg
const int DIO = A2; // 4seg
// must be pin 2 or 3 on nano for interrupts to work
const int STRT = 3; // start button and footswitch

const int RE_BUT = 6; // rotary encoder button  // NOTE:  library does pinmode pullup
const int RE_A = 7; // rotary encoder motion
const int RE_B = 8; // rotary encoder motion

const int BZR = 9; // buzzer or beeper



// --------    state variables -----------


enum en_states {
  EN_ACTIVE, //0
  EN_OVERRIDE_ON, //1
  EN_IDLE //2
};
en_states EN_STATE = EN_IDLE;
volatile bool EN_STATE_CHANGE = 0;
volatile en_states EN_ISR_REQ_STATE = EN_IDLE;

volatile int STRT_state = 0; // for button interrupt reads
const int STRT_DEBOUNCE_TIME = 200; // ms
volatile long unsigned last_start_debounce = 0; // store when timer finishes in ms 
//long unsigned start_pr_time = 0; // time elapsed since start pressed; reset at release

int cur_tmr_val = 5; // seconds; does not count down
int new_tmr_val = 5; // seconds, for when setting a new value
const int MAX_TIME = 60 * 60 -1; // 1 hour
long unsigned timer_end = 0; // store when timer finishes in ms 

//long unsigned last_disp_update = 0; // used for refreshing 47seg 

long unsigned buzz_end = 0; // store when buzzer finishes in ms 
const int BUZZ_LENGTH = 300; // time in ms for the buzzer/beeper

bool prev_RE_button = false;
bool prev_SLS = false;
bool prev_ENS = false;

int rot_buffer = 0;
long unsigned last_rot = 0;
int rot_delay = 100; // ms
long unsigned first_push = 0;
int push_cooldown = 300; // ms


enum f_states {
  F_SELECT,
  F_TIMER_EDIT
};
f_states F_STATE = F_SELECT;


int interval_len = 1000; //ms
long unsigned next_interval = 0;




TM1637 tm(CLK, DIO); // set up 47seg
Encoder encoder(RE_A, RE_B, RE_BUT); // set up rotary encoder
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display





// display a value given in seconds as mm:ss on the segment display
void displayTimeSeg(int seconds) {
  int m = seconds / 60;
  int s = seconds % 60;

  tm.point(1);
  tm.display(3, s % 10);
  tm.display(2, s / 10 % 10);
  tm.display(1, m % 10);
  tm.display(0, m / 10 % 10);
}

void displayNegTimeSeg(int seconds) {
  int m = seconds / 60;
  int s = seconds % 60;

  tm.point(1);
  tm.display(3, s % 10);
  tm.display(2, s / 10 % 10);
  tm.display(1, m % 10);
  tm.display(0, '-');
}


void enlargerOff() {
    Serial.println("Enlarger turned off!");
    digitalWrite(ENR, HIGH); // turn off enlarger
    //delay(500); // TODO: consider the wisdom of this decision
    digitalWrite(SLR, LOW); // turn on safelight
    lcd.backlight();
    //EN_STATE = EN_IDLE;
    displayTimeSeg(cur_tmr_val);
}


void enlargerOn() {
    Serial.println("Enlarger turned on!");
    digitalWrite(SLR, HIGH); // turn off safelight
    lcd.noBacklight();
    //delay(500); // TODO: consider the wisdom of this decision
    digitalWrite(ENR, LOW); // turn on enlarger
    //EN_STATE = EN_ACTIVE;
    displayTimeSeg(cur_tmr_val);   
}


void updDispTimerEdit(int val) {
    int s = val % 60;
    int m = (val % 3600) / 60;
    int h = val / 3600;
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


void buzz() {
  digitalWrite(BZR,HIGH);
  Serial.println("buzz on");
  buzz_end = millis() + BUZZ_LENGTH;
}

void buzzcheck() {
  if (buzz_end == 0) {
    return;
  } else if (buzz_end <= millis()) {
      buzz_end = 0;
      digitalWrite(BZR,LOW);
      Serial.println("buzz off");
  }
}


void start_isr() {

  // within the ISR and any functions it calls, time (as seen by delay() and millis() is frozen
  // you can call them, but they don't advance
  // Also, don't println from here...
  // https://forum.arduino.cc/t/delay-and-interrupts/405046
  


  //debounce - ok to use millis since time is frozen and we're not counting it to move
  unsigned long now = millis();
  if (now - last_start_debounce < STRT_DEBOUNCE_TIME) {
    last_start_debounce = now;
    return;
  }
  last_start_debounce = now;

  // read start pin
  STRT_state = digitalRead(STRT);
  // Serial.println("ISR: start is "+String(st));   // don't print from ISRs - bad mojo


  // timer triggers on release of start when in idle
  // if we detect another press while in active, stay in override until release
  // and on release cancel timer and turn off enlarger 

  // TODO maybe Long press just turns on enlarger without invoking timer if held down for at least X time???
  // strt_pr_time = millis();


  if (STRT_state == HIGH) { // then start has just been pressed
    
    if (EN_STATE == EN_ACTIVE) {  
      // if start is pressed while enlarger is active, that means
      //  we want to keep going past the timer
      EN_ISR_REQ_STATE = EN_OVERRIDE_ON;  
      EN_STATE_CHANGE = 1; 
    }
    
  } else if (STRT_state == LOW) { // start has been released
    
    if (EN_STATE == EN_IDLE) {
      // if start is released and we're idle, then start the timer
        EN_ISR_REQ_STATE = EN_ACTIVE;
        EN_STATE_CHANGE = 1;
    } else if (EN_STATE == EN_OVERRIDE_ON || EN_STATE == EN_ACTIVE ) {
      // if start is released while in override, this ends override
      // if start is released while in active, prematurely end timer
      EN_ISR_REQ_STATE = EN_IDLE;
      EN_STATE_CHANGE = 1;
    }
     
  }
}






  /*     ----------    S E T U P     --------- */


  void setup() {

    
    #ifdef DEBUG 
      Serial.begin(115200);
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
    pinMode(BZR, OUTPUT);

    

    // set up other hardware

    // 4-7 segment display
    tm.init();
    //set brightness; 0-7
    tm.set(2);
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

  buzz();
  }





  /*     ----------    L O O P     --------- */



  void loop() {

    long unsigned now = millis(); // ms

    #ifdef DEBUG
      if (next_interval < millis()) {
        next_interval = millis() + interval_len;
        Serial.println("================================="); 
      }
    #endif DEBUG

    buzzcheck(); //turn off buzzer if needed


    // do this even if we ignore to prevent buildup
    // read the debounced value of the encoder button
    bool pb = encoder.button();
    // get the encoder variation since our last check, it can be positive or negative, or zero if the encoder didn't move
    // only call this once per loop cicle, or at any time you want to know any incremental change
    int delta = encoder.delta();



    // since we can't delay or use millis in functions called by the ISR, let's use EN_STATE_CHANGE and do that all here
    if(EN_STATE_CHANGE == 1) {
      #ifdef DEBUG
        Serial.println("state change from ISR detected; old state is: "+String(EN_STATE));  
        Serial.println("new state is "+String(EN_ISR_REQ_STATE));
      #endif
      EN_STATE_CHANGE = 0;
      if (EN_ISR_REQ_STATE == EN_ACTIVE) {
        EN_STATE = EN_ACTIVE;
        buzz();
        enlargerOn();
        timer_end = now + 1000 * cur_tmr_val; // start the timer
      } else if (EN_ISR_REQ_STATE == EN_IDLE) {
        EN_STATE = EN_IDLE;
        buzz();
        enlargerOff();
        timer_end = 0;
      } else if (EN_ISR_REQ_STATE == EN_OVERRIDE_ON) {
        #ifdef DEBUG
          Serial.println("OVERRIDE on");
        #endif
        EN_STATE = EN_OVERRIDE_ON;
      }
    }


    // if we're in the middle of making an exposure...f
    if (EN_STATE == EN_ACTIVE || EN_STATE == EN_OVERRIDE_ON) {
        long unsigned timeleft = 0; // seconds
        long unsigned overtime = 0; // seconds
        if (timer_end >= now) { // timer still running
          timeleft = (timer_end - now) / 1000;
          overtime = 0;
        } else {
          timeleft = 0;
          overtime = (now - timer_end) / 1000 + 1; // +1 to avoid spending two seconds at zero
          #ifdef DEBUG
            Serial.println("past end of timer");
          #endif
        }
        
        if (timeleft >= 0 && overtime == 0) {  // if timer still running
          //if (now > 1000 + last_disp_update) {
            displayTimeSeg(timeleft);
            //last_disp_update = now;
          //}
        } 
        if (timeleft == 0 && overtime > 0  && EN_STATE != EN_OVERRIDE_ON) {
            // timer has finished and not in override
            EN_STATE = EN_IDLE;
            enlargerOff();
            buzz();
            timer_end = 0;
            #ifdef DEBUG
              Serial.println("Timer end detected");
            #endif
        } else if (EN_STATE == EN_OVERRIDE_ON) {
             if (overtime > 0) {
               displayNegTimeSeg(overtime);
             } else {
               displayTimeSeg(timeleft);
             }
             #ifdef DEBUG
               Serial.println("override state detected");
             #endif
        }
   
    return;  // IGNORE REST OF LOOP
    }

    
    // enlarger inactive - pay attention to switches and RE now

    // handle manual switch states
    // remember that the switches active high but the relays are active low
    digitalWrite(ENR, !digitalRead(ENS));
    digitalWrite(SLR, !digitalRead(SLS));



    // pay attention to re button if the last time it was pressed was before cooldown ms ago
    // otherwise, we're in cooldown or pb isn't being pressed, so ignore
    if (pb && (first_push + push_cooldown < now)) {
      first_push = now;
      f_states NEW_F_STATE = F_STATE;
      switch(F_STATE) {
      
          case F_SELECT: {
            // button press => enter time set
            
            // enter function settings
            NEW_F_STATE = F_TIMER_EDIT;
            updDispTimerEdit(cur_tmr_val);
            #ifdef DEBUG
              Serial.println("button press => enter set time");
            #endif
            break;
     

            // rotate => select different function
            // TODO
          }

          case F_TIMER_EDIT: {
            // button press => go back to select function state

            // return to F_SELECT
            cur_tmr_val = new_tmr_val;
            displayTimeSeg(cur_tmr_val);
            updDispFTimer();  
            NEW_F_STATE = F_SELECT;
            #ifdef DEBUG
              Serial.println("button press: update 4s7d and back to f-select");
            #endif
            break;
        }
  
      } // end F_STATE switch

      F_STATE = NEW_F_STATE;  
    }

  
  //  prev_RE_button = pb;

    // rotate => set new timer value
    if(delta != 0) {
      rot_buffer -= delta;
      last_rot = now;
      #ifdef DEBUG
        Serial.println("rb = "+String(rot_buffer) +"   delta = "+String(delta));
      #endif
    } 
    long unsigned lr = last_rot;
 
     
    // if its been more than rot_delay since last rotary action
    if (F_STATE == F_TIMER_EDIT && (now > lr + rot_delay)) {
      int rb = rot_buffer;
      rot_buffer = 0;
      // some time has passed since the rotary did anything
      String dir = "Neutral";
      if (rb > 0) {
        new_tmr_val = min(new_tmr_val + 1, MAX_TIME);
        updDispTimerEdit(new_tmr_val);
        #ifdef DEBUG
          dir = "Right  ";
          Serial.println(dir + ":  rb = "+String(rb) +"   new tmr val = "+String(new_tmr_val));
        #endif
      } else if (rb < 0) {
        new_tmr_val = max(new_tmr_val - 1, 1);
        updDispTimerEdit(new_tmr_val);
        #ifdef DEBUG
          dir = "Left   ";
          Serial.println(dir + ":  rb = "+String(rb) +"   new tmr val = "+String(new_tmr_val));
        #endif
      } else {
        #ifdef DEBUG
          //Serial.println("rb is neutral after delay");
        #endif
      }
    } else if (F_SELECT && (now > lr + rot_delay)) {
      rot_buffer = 0;
    }
    

  } // end loop





  /* RESOURCES

      pinout:
      https://i.stack.imgur.com/W9Ril.png

      Power supply wiring:
      https://electronics.stackexchange.com/questions/60199/powering-arduino-nano-12volts

      RE:
      https://github.com/John-Lluch/Encoder
        https://www.instructables.com/Improved-Arduino-Rotary-Encoder-Reading/
        https://github.com/John-Lluch/Encoder/issues/4#issue-955354998
        https://github.com/PaulStoffregen/Encoder (didn't use)
        

      Button interrupts:
      https://www.allaboutcircuits.com/technical-articles/using-interrupts-on-arduino/
        rising falling
        https://forum.arduino.cc/index.php?topic=147825.0
        pull up pull down
        https://www.instructables.com/Understanding-the-Pull-up-Resistor-With-Arduino/
        debouncing
        https://forum.arduino.cc/t/debouncing-an-interrupt-trigger/45110/3
        https://github.com/thomasfredericks/Bounce2   <<<<< TODO:  seriously consider trying this!!!

      TM1637 4 segment display
      https://github.com/Seeed-Studio/Grove_4Digital_Display
      https://create.arduino.cc/projecthub/ryanchan/tm1637-digit-display-arduino-quick-tutorial-ca8a93
      https://www.makerguides.com/tm1637-arduino-tutorial/

      relay
      https://randomnerdtutorials.com/guide-for-relay-module-with-arduino/

      enum switch
      https://stackoverflow.com/questions/52932529/why-does-my-switch-case-default-when-using-enums

      
      
      USB Mini male plug pinout
      
      |_|     |_|
      |_|_____|_|
          | |   (top row when narrow side of connector is up)
        |  |  | (bottom row when narrow side of connector is up)
      GND     5v 


RE wiring
                    ____
      RE_A (7)   - |    |_  RE_BUT (6)
      GND        - |    |_  GND
      RE_B (8)   - |____|

     Use 10k pullups on RE_A and RE_B to 5v (if not using KY040 module)


     */
