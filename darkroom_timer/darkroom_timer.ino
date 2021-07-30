#include "Arduino.h" 
#include <TM1637.h>
#include "Encoder.h"
#include <LiquidCrystal_I2C.h>




/*

  For use on Arduino Nano

 */


#define DEBUG


// --------- pins -----------

//arduino pin mapping 
//see https://www.electronicshub.org/wp-content/uploads/2021/01/Arduino-Nano-Pinout.jpg
#define D4 PD4
#define D5 PD5
#define D3 PD3
#define D6 PD6
#define D7 PD7
#define D8 PB0
//#define D9 PB1

// enlarger and safelight switches and relays
const int ENS = D4; // enlarger switch, active high, pulldown
const int SLS = D5; // safelight switch, active high, pulldown
const int ENR = A0; // enlarger relay, active low, pullup
const int SLR = A1; // safelight relay, active low, pullup
const int CLK = A3; // 4seg
const int DIO = A2; // 4seg
// must be pin 2 or 3 on nano for interrupts to work
const int STRT = D3; // start button and footswitch

const int RE_BUT = D6; // rotary encoder button  // NOTE:  library does pinmode pullup
const int RE_A = D7; // rotary encoder motion
const int RE_B = D8; // rotary encoder motion

const int BZR = 9; // buzzer or beeper



// --------    state variables -----------

int cur_tmr_val = 5; // seconds; does not count down
int new_tmr_val = 5; // seconds, for when setting a new value
enum en_states {
  EN_ACTIVE, //0
  EN_OVERRIDE_ON, //1
  EN_IDLE //2
};
en_states EN_STATE = EN_IDLE;
volatile bool EN_STATE_CHANGE = 0;
volatile en_states EN_ISR_REQ_STATE = EN_IDLE;

volatile int STRT_state = 0; // for button interrupt reads
const int STRT_DEBOUNCE_TIME = 50; // ms
volatile long unsigned last_start_debounce = 0; // store when timer finishes in ms 
//long unsigned start_pr_time = 0; // time elapsed since start pressed; reset at release

long unsigned timer_end = 0; // store when timer finishes in ms 
long unsigned last_disp_update = 0; // used for refreshing 47seg 
long unsigned buzz_end = 0; // store when buzzer finishes in ms 

const int MAX_TIME = 60 * 60 -1; // 1 hour
const int BUZZ_LENGTH = 300; // time in ms for the buzzer/beeper

enum f_states {
  F_SELECT,
  F_TIMER_EDIT
};
f_states F_STATE = F_SELECT;


bool prev_RE_button = false;
bool prev_SLS = false;
bool prev_ENS = false;



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

//    #ifdef DEBUG 
//      Serial.print("EN_STATE: ");
//      Serial.println(EN_STATE);
//      Serial.print("F_STATE: ");
//      Serial.println(F_STATE);
//    #endif
    buzzcheck(); //turn off buzzer if needed


    // since we can't delay or use millis in functions called by the ISR, let's use EN_STATE_CHANGE and do that all here
    if(EN_STATE_CHANGE == 1) {
      Serial.println("state change from ISR detected; old state is: "+String(EN_STATE));  
      Serial.println("new state is "+String(EN_ISR_REQ_STATE));
      EN_STATE_CHANGE = 0;
      if (EN_ISR_REQ_STATE == EN_ACTIVE) {
        EN_STATE = EN_ACTIVE;
        buzz();
        enlargerOn();
        timer_end = millis() + 1000 * cur_tmr_val; // start the timer
      } else if (EN_ISR_REQ_STATE == EN_IDLE) {
        EN_STATE = EN_IDLE;
        buzz();
        enlargerOff();
        timer_end = 0;
      } else if (EN_ISR_REQ_STATE == EN_OVERRIDE_ON) {
        Serial.println("OVERRIDE on");
        EN_STATE = EN_OVERRIDE_ON;
      }
    }


    // if we're in the middle of making an exposure...
    if (EN_STATE == EN_ACTIVE || EN_STATE == EN_OVERRIDE_ON) {
        long unsigned now = millis(); // ms
        long unsigned timeleft = 0; // seconds
        long unsigned overtime = 0; // seconds
        if (timer_end >= now) { // timer still running
          timeleft = (timer_end - now) / 1000;
          overtime = 0;
        } else {
          timeleft = 0;
          overtime = (now - timer_end) / 1000 + 1; // +1 to avoid spending two seconds at zero
          Serial.println("past end of timer");
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
            Serial.println("Timer end detected");
        } else if (EN_STATE == EN_OVERRIDE_ON) {
             if (overtime > 0) {
               displayNegTimeSeg(overtime);
             } else {
               displayTimeSeg(timeleft);
             }
             Serial.println("override state detected");
        }
   
    return;  // IGNORE REST OF LOOP
    }

    
    // enlarger inactive - pay attention to switches and RE now

    // handle manual switch states
    // remember that the switches active high but the relays are active low
    digitalWrite(ENR, !digitalRead(ENS));
    digitalWrite(SLR, !digitalRead(SLS));


    // if nothing's happened on the RE, skip rest
    if (!pb && delta == 0) {
      return;
    }

    switch(F_STATE) {

      case F_SELECT: {
        // button press => enter time set
        if (pb && (pb != prev_RE_button) ) {
          // enter function settings
          F_STATE = F_TIMER_EDIT;
          updDispTimerEdit(cur_tmr_val);
          Serial.println("button press => enter set time");
        }

        // rotate => select different function
        // TODO
      }

      case F_TIMER_EDIT: {
        // button press => go back to select function state
        if (pb && (pb != prev_RE_button) ) {
          // return to F_SELECT
          cur_tmr_val = new_tmr_val;
          updDispFTimer();  
          F_STATE = F_SELECT;
          Serial.println("button press: back to f-select");
        }
        
        // rotate => set new timer value
        int x = new_tmr_val;
        if (delta >= 1) {
          x += 1;
        } else if (delta <= -1) {
          x -= 1;
        } else {
          return; // should not happen
        }
        if (x > 0 && x < MAX_TIME) {
          new_tmr_val = x;
          updDispTimerEdit(new_tmr_val);
          Serial.println("delta = " + String(delta) + " new_tmr_val = "+String(new_tmr_val));
          Serial.println("rotation => time adjust");
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
