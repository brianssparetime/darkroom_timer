#include "Arduino.h" 
#include <TM1637.h>
#include "Encoder.h"
#include <LiquidCrystal_I2C.h>






#define DEBUG



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

const int RE_BUT = D6; // rotary encoder button  // NOTE:  library does pinmode pullup
const int RE_A = D7; // rotary encoder motion
const int RE_B = D8; // rotary encoder motion


const int BZR = 9; // buzzer or beeper



volatile int STRT_state = 0; // for button interrupt reads
const int BUZZ_LENGTH = 1200; // hz freq for the buzzer/beeper
long unsigned buzz_end = 0; // store when buzzer finishes in ms 
bool buzz_trigger = 0;
int STRT_DEBOUNCE_TIME = 50; // ms
long unsigned last_start_debounce = 0; // store when timer finishes in ms 


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





void start_isr() {

  //debounce - ok to use millis since time is frozen and we're not counting it to move
  unsigned long now = millis();
  if (now - last_start_debounce < STRT_DEBOUNCE_TIME) {
    last_start_debounce = now;
    return;
  }
  last_start_debounce = now;

  
  STRT_state = digitalRead(STRT);
  if (STRT_state == HIGH) { // then start has just been pressed
    buzz_trigger = 1;    
  } 
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



  /*     ----------    S E T U P     --------- */


  void setup() {
    Serial.begin(9600);
    Serial.println("Timer online");

    // set up pins
    pinMode(BZR, OUTPUT);
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
    tm.set(7);

    tm.point(1);
    tm.display(3, 3);
    tm.display(2, 2);
    tm.display(1, 1);
    tm.display(0, 0);

    // rotary encoder
    EncoderInterrupt.begin( &encoder );

    // initialize the lcd 
    lcd.init();                      
    // Print a message to the LCD.
    lcd.backlight();
    lcd.setCursor(0,0); // col, row)
    lcd.print("  TIMER");

  }


  /*     ----------    L O O P     --------- */



  void loop() {

    if (buzz_trigger == 1) {
      buzz_trigger = 0;
      Serial.println("start pressed - trying to buzz");
      buzz();
    }
    buzzcheck();

    // do this even if we ignore to prevent buildup

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

    // handle manual switch states
    // remember that the switches active high but the relays are active low
    digitalWrite(ENR, !digitalRead(ENS));
    digitalWrite(SLR, !digitalRead(SLS));

 



  }




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
