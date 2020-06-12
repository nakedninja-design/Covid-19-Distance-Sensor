/*
 * Distance measuring and notification
 * 
 * Author: Naked Ninja
 * For products made by Naked Ninja
 * Website: https://www.NakedNinja.cc
 *
*/

#include <Adafruit_NeoPixel.h>  // Library for RGB LED strip
#include <NewPing.h>            // Library for Ultrasonic Sensor

// Pin definitions
#define LED           4   // LED
#define US_TRIG       9   // Ultrasonic Sensor trigger pin
#define US_ECHO       10  // Ultrasonic Sensor echo pin
#define RGB_LED1      11  // RGB LED strip data pin
#define RGB_LED2      12  // RGB LED strip data pin

// Other definitions
#define RGB_COUNT         11  // Amount of RGB LEDS on the strip
#define RGB_BRIGHTNESS    100 // Set RGB brightness level
#define MEASUREMENT_DELAY 200 // Measurement loop time (minimal value of 29~50)
#define MIN_DISTANCE      150 // RGB strip will be RED if the distance measured is lower then this value
#define MAX_DISTANCE      200 // If motion is detected, RGB strip will be GREEN if the distance measured is lower then this value
// Maximum sensor distance is rated at 400-500cm.

// Create RGB LED strip object
Adafruit_NeoPixel rgb1(RGB_COUNT, RGB_LED1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rgb2(RGB_COUNT, RGB_LED2, NEO_GRB + NEO_KHZ800);

// Create Ultrasonic Sensor object
NewPing sonar(US_TRIG, US_ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Color variables
uint32_t red    = rgb1.Color(255, 0,   0);
uint32_t yellow = rgb1.Color(255, 255, 0);
uint32_t green  = rgb1.Color(0,   255, 0);

// Global variables
int distance = 0;
volatile uint32_t color = green;
volatile int i = -1;


void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  rgb1.begin();  // Begin RGB communication
  rgb1.setBrightness(RGB_BRIGHTNESS); // Set brightness
  rgb1.clear();  // Set entire RGB strip to black (off)
  rgb1.show();   // Show set color on RGB strip

  rgb2.begin();  // Begin RGB communication
  rgb2.setBrightness(RGB_BRIGHTNESS); // Set brightness
  rgb2.clear();  // Set entire RGB strip to black (off)
  rgb2.show();   // Show set color on RGB strip

  Serial.begin(115200); // Start serial communication
  Serial.println("start");

  // initialize timer1
  noInterrupts();   // disable all interrupts
  TCCR1A = 0;       // Reset TCCR1A
  TCCR1B = 0;       // Reset TCCR1B
  TCNT1  = 0;       // Initialize counter value to 0

  TCCR1B |= (1 << WGM12);   // CTC Mode
  TCCR1B |= (1 << CS12);    // Prescaler 1024
  TCCR1B |= (1 << CS10);    // Prescaler 1024
  OCR1A = 31249;            // F = 0.5Hz => OCR1A = 16MHz / (1024*0.5) - 1
  TIMSK1 |= (1 << OCIE1A);  // Output Compare Match A Interrupt Enable
  interrupts();             // Enable all interrupts

  /*
  F = (F_clk)/ (Prescaler*(1+OCR1A))
  
  OCR1A = F_clk / (Prescaler*F) - 1
    Value must be smaller than 65536
    where F = frequency of timer required
    example: frequency required is 1000Hz, 16MHz clk frequency and prescaler set to 64:
      OCR1A = 16MHz / (64*1000Hz) - 1 = 249
      f = 
      
  TCCR1A
    Bit:  7       6       5       4       3       2     1     0
          COM1A1  COM1A0  COM1B1  COM1B0  –       –     WGM11 WGM10 
  TCCR1B
    Bit:  7       6       5       4       3       2     1     0
          ICNC1   ICES1   –       WGM13   WGM12   CS12  CS11  CS10        
  TIMSK1
    Bit:  7       6       5       4       3       2       1       0
          –       –       ICIE1    –       –       OCIE1B  OCIE1A  TOIE1  

  OCR1A: 
    The given value for OCR1A is the TOP value the counter needs to hit before the interrupt flag is set. 
  Mode:
    To change the timer mode, from normal operation to for example clear timer on compare (CTC) the following
    values need to be changed: TCCR1A & TCCR1B
    For CTC we need mode 4 which is setting WGM12 bit high on the TCCR1B.
    (For other modes see datasheet)
      
  Prescaler:
    To change the prescaler, set TCCR1B with the following values:
      CS12  CS11  CS10  Description
       0     0     0    No clock source (Timer/Conter Stopped)
       0     0     1    clk/1 (no prescaling)
       0     1     0    clk/8 (from prescaler)
       0     1     1    clk/64 (from prescaler)
       1     0     0    clk/256 (from prescaler)
       1     0     1    clk/1024 (from prescaler)
       1     1     0    External clock source on T1 pin. Clock on falling edge.
       1     1     1    External clock source on T1 pin. Clock on rising edge.
    Example of setting these prescalers:
      TCCR1B |= (1 << CS10);
      TCCR1B |= (1 << CS12);

  Interrupt mask register:
    OCIE1A: Output Compare A Match Interrupt Enable
      When this bit is set, and the I-flag in status resigster is set (interrupt globally enabled), the timer/counter1
      output compare A match interrupt is enabled.The corresponding interrupt vector is executed when
      the OCF1A flag, located in TIFR1, is set.
  */
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  if (color == green)
    isr_colorWipeOutToIn();
  else
    isr_colorWipeInToOutTo();
}

void loop()
{
  distance = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.print(distance); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");

  switch (distance) {
    case 1 ... MIN_DISTANCE:
      Serial.println("red");
      if (color == green)
        i = -1;
      color = red;
      break;
    default:
      Serial.println("green");
      if (color == red)
        i = -1;
      color = green;
      break;
  }

  delay(MEASUREMENT_DELAY);
}

void colorBlink(uint32_t color, int times, int wait) {
  for (int i = 0; i < times; i++) {
    rgb1.fill(yellow, 0, RGB_COUNT);
    rgb2.fill(yellow, 0, RGB_COUNT);
    rgb1.show();  // Show set color on RGB strip
    rgb2.show();  // Show set color on RGB strip
    delay(wait);

    rgb1.clear();
    rgb2.clear();
    rgb1.show();  // Show set color on RGB strip
    rgb2.show();  // Show set color on RGB strip
    delay(wait);
  }
}

void isr_colorWipeOutToIn() {
  if (i == -1) {
    rgb1.clear(); // Clear RGB strip
    rgb2.clear(); // Clear RGB strip
    rgb1.show();  // Show set color on RGB strip
    rgb2.show();  // Show set color on RGB strip

    i = RGB_COUNT;
    return;
  }

  rgb1.setPixelColor(i, color); //  Set pixel's color (in RAM)
  rgb2.setPixelColor(i, color); //  Set pixel's color (in RAM)
  rgb1.show();                  //  Update strip to match
  rgb2.show();                  //  Update strip to match

  i--;

  if (i == 0) {
    i = -1;
  }
}

void isr_colorWipeInToOutTo() {
  if (i == -1) {
    rgb1.clear(); // Clear RGB strip
    rgb2.clear(); // Clear RGB strip
    rgb1.show();  // Show set color on RGB strip
    rgb2.show();  // Show set color on RGB strip

    i = 0;
    return;
  }

  rgb1.setPixelColor(i, color); //  Set pixel's color (in RAM)
  rgb2.setPixelColor(i, color); //  Set pixel's color (in RAM)
  rgb1.show();                  //  Update strip to match
  rgb2.show();                  //  Update strip to match

  i++;

  if (i == RGB_COUNT) {
    i = -1;
  }
}
