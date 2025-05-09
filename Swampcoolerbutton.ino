#include <LiquidCrystal.h> 
#include "DHT.h" // "DHT Sensor Library" by Adafruit
#include <Stepper.h>
#include <Wire.h>
#include "RTClib.h" // "RTClib" by Adafruit
// May need "Adafruit BusIO" as additional dependency

// Environmental threshold variables
float tempThreshold = 78;  // Temperature threshold
float waterThreshold = 500; // Water level threshold

// Stepper motor configuration
const int stepsPerRevolution = 2048;  // Number of steps for a full revolution
const int rpm = 10;  // Rotations per minute
Stepper stepper(stepsPerRevolution, 15, 17, 16, 18);  // Initialize the stepper library on pins 8 through 11

// DHT sensor setup
#define DHTPIN 14  // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // Set DHT sensor type
DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor

// LCD display setup
LiquidCrystal lcd(11, 12, 2, 3, 4, 5);  // Set the LCD pins

// Clock variables
RTC_DS1307 rtc;  // Real Time Clock object
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Fan control pins
#define enA 22
#define in1 23
#define in2 25

// ADC (Analog-to-Digital Converter) setup // NOT USED?
#define RDA 0x80
#define TBE 0x20 

// Water sensor
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0; //serialControlRegA
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1; //serialControlRegB
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2; //serialControlRegC
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4; //baudRateReg
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6; //dataReg
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A; //adcControlStatusA
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B; //adcControlStatusB
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78; //adcData

// Timer code?
// volatile unsigned char *timerControlRegC = (unsigned char *) 0x82;
// volatile unsigned char *timerInterruptMask1 = (unsigned char *) 0x6F;

// long value = 0UL;
// long maxtime = 160000000UL;

// Timer configuration (used in MyDelay function)
volatile unsigned char* timerControlRegA = (unsigned char*) 0x80; // TCCR1A
volatile unsigned char* timerControlRegB = (unsigned char*) 0x81; // TCCR1B
volatile unsigned int* timerCounter1 = (unsigned int*) 0x84; // TCNT1
volatile unsigned char* timerInterruptFlagReg1 = (unsigned char*) 0x36; // TIFR1
// int oneSecondTicks = 62500;  // Ticks value for 1 second delay with timer
// int freq = 0;

// Port B registers (Digital I/O pins) (used for LEDs)
volatile unsigned char* portBData = (unsigned char*) 0x25; // Port B data register
volatile unsigned char* portBDirection = (unsigned char*) 0x24;  // Port B Data Direction Register
volatile unsigned char* portBInput = (unsigned char*) 0x23;  // Port B Input Pin Address

// Used to save which state program is in?
int toggle = 0;
int state = 0;
int currentState = 0;

// Reset button (bottom) - A15 (PK7), Clockwise Vent (middle) - A14 (PK6), Counterclockwise Vent (top) - A13 (PK5)
// Opens analog for ports Port K0-7
volatile unsigned char* port_k = (unsigned char*) 0x108; 
volatile unsigned char* ddr_k  = (unsigned char*) 0x107; 
volatile unsigned char* pin_k  = (unsigned char*) 0x106; 
bool reset = false;

unsigned long previousMillis = 0;

// Function declarations
void adc_init(); // used for water sensor
unsigned int adc_read(unsigned char adc_channel); // used for water sensor
void MyDelay(unsigned int freq); // replaces regular delay function
void TimerDelay(unsigned int ticks); // UNKNOWN?
void U0putchar(unsigned char U0pdata); // replaces serial, only one char at a time
// void U0init(unsigned long U0baud);
int WaterSensor();
void ClockModule(); // Displays the current date and time on the serial monitor.
void FanControl(); 
void VentControl(); 
void LCDError(); // Prints error message to display
void LCDData(float h, float f); // Prints humidity and temperature to display
double DHTSensor(); // Read humidity and temperature, prints to display
// Different states for system
void ErrorState(); 
void RunningState(); 
void IdleState();
void DisabledState();
void FanON(bool on);

void RTCErrors(int e);

void setup() {
  Serial.begin(9600);

  // Initialize RTC clock
  if (!rtc.begin()) {
    RTCErrors(0); // "Couldn't find RTC"
    Serial.flush(); // Not sure if can use?
    while (1) delay(10);  // Infinite loop if RTC not found
  }

  if (!rtc.isrunning()) {
    RTCErrors(1); // "RTC is NOT running, setting the time"
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Set RTC to compile time
  }

  // Initialize LCD
  lcd.begin(16, 2);

  // Initialize temperature and humidity sensor
  dht.begin();

  // Initialize timers
  *timerInterruptFlagReg1 = B00000000;
  *timerCounter1 = B00000000;
  *timerControlRegB = B00000000;
  *timerControlRegA = B00000000;

  // Initialize ADC and UART (for water sensor)
  U0init(9600);
  adc_init();

  // Initialize digital output pins
  // *portBDirection = B10001111; // stops LCD from working

  // Initialize analog input pins
  // *portKDirection = B01111111;

  // Initialize LED pins
  pinMode(52, OUTPUT); // RED
  pinMode(50, OUTPUT); // BLUE
  pinMode(53, OUTPUT); // GREEN
  pinMode(51, OUTPUT); // YELLOW

  // Initalize fan pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // Start with fan off
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  // Initialize Reset Button
  *ddr_k &= 0b10000000; // since using PK7, create mask value to get PK7
  *ddr_k &= 0b01000000; // Clockwise vent control button
  *ddr_k &= 0b00100000; // Counterclockwise vent control button

}

void loop() {
  // Main program loop
  // Check and handle system states based on sensor inputs and toggle switch

  DelayFunction(300);

  // Change states based off buttons or environmental conditions
  // 0 = initial, 1 = disabled, 2 = idle, 3 = running, 4 = error
  // Initial, set it to disabled state
  if(toggle == 0){ // sets to disabled state when first powered on
    lcd.clear();
    DisabledState();
  }
  // In disabled state
  if((toggle == 0 || toggle == 1) && (*pin_k & 0b10000000)){ // when disabled and button is pressed, set to running state
    lcd.clear();
    IdleState();
  }
  // In idle state
  if(toggle == 2 && WaterSensor() <= waterThreshold){ // when idle and water below threshold, set to error state
    lcd.clear();
    ErrorState();
  }
  if(toggle == 2 && DHTSensor() > tempThreshold){
    lcd.clear();
    RunningState();
  }
  if(toggle == 2 && (*pin_k & 0b10000000)){
    lcd.clear();
    DisabledState(); // somehow returns to disabled state
  }
  // In running state
  if(toggle == 3 && DHTSensor() <= tempThreshold){
    lcd.clear();
    IdleState();
  }
  if(toggle == 3 && WaterSensor() < waterThreshold){ // when running an dwater above threshold, set to idle state
    lcd.clear();
    ErrorState();
  }
  if(toggle == 3 && (*pin_k & 0b10000000)){
    lcd.clear();
    DisabledState();
  }
  // In error state
  if(toggle == 4 && (*pin_k & 0b10000000) && WaterSensor() > waterThreshold){
    lcd.clear();
    IdleState();
  }

  // Loop states otherwise
  if(toggle == 1){
    DisabledState();
  }
  if(toggle == 2){
    IdleState();
  }
  if(toggle == 3){
    RunningState();
  }
  if(toggle == 4){
    ErrorState();
  }
}

// Disabled State Function
// This state represents the system being turned off or disabled.
// Yellow LED on, water and humidity are not monitored, system can be start using button
void DisabledState() {
  if (toggle != 1) {
    toggle = 1;
    // lcd.clear();
    *portBData &= B01111111;
    *portBData |= B00000010;
    *portBData &= B11111110;
    *portBData &= B11111011;
    *portBData &= B11110111;

    SerialStates(1);
    ClockModule();
  }

  PORTB &= ~(1 << 1); // Clear bit (RED)
  PORTB &= ~(1 << 3); // Clear bit (BLUE)
  PORTB &= ~(1 << 0); // Clear bit ((GREEN))
  PORTB |= (1 << 2); // Set bit 2 to HIGH (YELLOW)

  lcd.setCursor(0, 0);
  lcd.print("System Off");
  FanON(false);
}

// All other states
// System updates humidity and temperature on LCD once a minute. Stop button will turn off fan (if on) and set system to disabled state. Vent is controlled through buttons

// Idle State Function
// This state is active when the system is on but idle due to no need for intervention.
// Green LED on, water level is monitored and transitions to error state if too low, time is displayed on LCD
void IdleState() {
  if (toggle != 2) {
    toggle = 2;
    // lcd.clear();
    *portBData &= B01111111;
    *portBData |= B00000001;
    *portBData &= B11111101;
    *portBData &= B11111011;
    *portBData &= B11110111;

    SerialStates(2);
    ClockModule();
  }

  PORTB &= ~(1 << 1); // Clear bit (RED)
  PORTB &= ~(1 << 2); // Clear bit (YELLOW)
  PORTB &= ~(1 << 3); // Clear bit (BLUE)
  PORTB |= (1 << 0); // Set Green HIGH (GREEN)

  DHTSensor();  // Read humidity and temperature, prints to display
  VentControl();  // Control ventilation based on environmental data
  FanON(false);
}

// Running State Function
// This state is active when the system needs to run due to environmental conditions.
// Blue LED on, temperature is monitored and transitions to idle if temperature is too low, water level is monitored and transitions to error if water too low
void RunningState() {
  if (toggle != 3) {
    toggle = 3;
    lcd.clear();
    *portBData |= B00001000;
    *portBData &= B11111101;
    *portBData &= B11111011;
    *portBData &= B11111110;
    *portBData |= B10000000;  // Turn on the fan

    SerialStates(3);
    ClockModule();
  }

  PORTB &= ~(1 << 1); // Clear bit (RED)
  PORTB &= ~(1 << 2); // Clear bit (YELLOW)
  PORTB &= ~(1 << 0); // Clear bit (GREEN)
  PORTB |= (1 << 3); // Set Blue HIGH (BLUE)

  DHTSensor();
  FanON(true);
  VentControl();
}

// Error State Function
// This state is triggered when there is an error, such as low water level.
// Red LED on, motor is turned off, error message is displayed, if water is above threshold when reset button is pressed then transition to idle state
void ErrorState() {
  if (toggle != 4) {
    toggle = 4;
    // lcd.clear();
    *portBData &= B01111111;
    *portBData |= B00000100;
    *portBData &= B11110111;
    *portBData &= B11111101;
    *portBData &= B11111110;

    SerialStates(4);
    ClockModule();
  }

  PORTB &= ~(1 << 2); // Clear bit (YELLOW)
  PORTB &= ~(1 << 0); // Clear bit ((GREEN))
  PORTB &= ~(1 << 3); // Clear bit (BLUE)
  PORTB |= (1 << 1); // Set bit HIGH (RED)

  VentControl();
  LCDError();  // Display error message on LCD
  FanON(false);
}

// DHT Sensor Function
// Reads and returns temperature from the DHT sensor, and updates the LCD.
double DHTSensor() {
  float humidity = dht.readHumidity();
  float temperatureF = dht.readTemperature(true); // true sets temperature to F

  // Check for failed reading from sensor
  if (isnan(humidity) || isnan(temperatureF)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return -1;  // Return error code if reading fails
  }

  LCDData(humidity, temperatureF);  // Update LCD with temperature and humidity data
  return temperatureF;  // Return the temperature reading
}

// LCD Data Display Function
// Displays humidity and temperature data on the LCD.
void LCDData(float humidity, float temperatureF) {
  lcd.setCursor(0, 0);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(temperatureF);
  lcd.print(" F");
}

// LCD Error Display Function
// Displays an error message on the LCD, typically for low water level.
void LCDError() {
  lcd.setCursor(0, 0);
  lcd.print("ERROR: LOW WATER");
}

// Ventilation Control Function
// Controls the stepper motor for ventilation based on sensor inputs.
void VentControl() {
  int steps = stepsPerRevolution / 30;  // Calculate steps for partial revolution
  stepper.setSpeed(5);
  if(*pin_k & 0b01000000){
    stepper.step(steps);
  }
  if(*pin_k & 0b00100000){ // Counterclockwise vent control button)
    stepper.step(-steps);
  }
  DelayFunction(300);
}

// Clock Module Function
// Displays the current date and time on the serial monitor.
void ClockModule() {
  DateTime now = rtc.now();

  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();
  char time[26] = {
    'a',
    't',
    ' ',
    hour / 10 + '0',
    hour % 10 + '0',
    ':',
    minute / 10 + '0',
    minute % 10 + '0',
    ':',
    second / 10 + '0',
    second % 10 + '0',
    ' ',
    'o',
    'n',
    ' ',
    month / 10 + '0',
    month % 10 + '0',
    '/',
    day / 10 + '0',
    day % 10 + '0',
    '/',
    (year / 1000) + '0',
    (year % 1000 / 100) + '0',
    (year % 100 / 10) + '0',
    (year % 10) + '0', ' '
  };
  for (int i = 0; i < 26; i++){
    U0putchar(time[i]);
  }
  U0putchar('\n');
}

// Timer Delay Function
// Creates a delay based on timer ticks.
void TimerDelay(unsigned int ticks) {
  *timerControlRegB &= 0x00;  // Clear timer control register B
  *timerControlRegA &= 0x00;  // Clear timer control register A
  *timerCounter1 = (unsigned int)(65535 - ticks);  // Set timer count
  *timerControlRegB |= 0b00000100;  // Set timer settings
  while ((*timerInterruptFlagReg1 & 0x01) == 0);  // Wait for timer overflow
  *timerControlRegB &= 0x00;  // Clear timer control register B again
  *timerInterruptFlagReg1 |= 0x01;  // Clear timer interrupt flag
}

void DelayFunction(unsigned long delayTime) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= delayTime) {
    previousMillis = currentMillis;
  } else {
    while(currentMillis - previousMillis < delayTime){
        currentMillis = millis();
    }
    previousMillis = currentMillis;
  }
}

void MyDelay(unsigned int freq){
  double period = 1.0/double(freq); // calc period
  double half_period = period/ 2.0f; // 50% duty cycle
  double clk_period = 0.0000000625; // clock period def
  unsigned int ticks = (half_period / clk_period); // calc ticks

  // *timerControlRegB &= 0xF8; // stop the timer
  // *timerCounter1 = (unsigned int) (65536 - ticks); // set the counts
  // // *timerControlRegA = 0; 
  // *timerControlRegB |= 0x01; // start the timer

  // while((*timerInterruptFlagReg1 & 0x01)==0); // wait for overflow
  // *timerControlRegB &= 0xF8; // stop the timer    
  // *timerInterruptFlagReg1 |= 0x01; // reset TOV

  *timerControlRegB &= 0b11111000; // set prescalar to 000
  // set the counts
  *timerCounter1 = (unsigned int) (65536 - ticks); 

  // start the timer
  *timerControlRegB |= 0b00000001;
  // wait for overflow
  while((*timerInterruptFlagReg1 & 0b00000001)==0); 
  // stop the timer
  *timerControlRegB &= 0b11111000;   
  // reset TOV
  *timerInterruptFlagReg1 |= 0b00000001;
}

// Water Sensor Function
// Reads and returns the water level from the ADC.
int WaterSensor() {
  adc_init();  // Initialize ADC
  int waterLevel = adc_read(0);  // Read water level from ADC channel 0
  return waterLevel;  // Return the water level reading
}

// Water sensor functions
// ADC Initialization Function
// Sets up the ADC for reading.
void adc_init() {
  // setup the A register
  // set bit   7 to 1 to enable the ADC
  *my_ADCSRA |= 0b10000000;
  // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b10111111;
  // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11011111;
  // clear bit 0-2 to 0 to set prescaler selection to slow reading
  *my_ADCSRA &= 0b11111000;
  // setup the B register
  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11110111;
  // clear bit 2-0 to 0 to set free running mode
  *my_ADCSRB &= 0b11111000;
  // setup the MUX Register
  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX &= 0b01111111;
  // set bit 6 to 1 for AVCC analog reference
  *my_ADMUX |= 0b01000000;
  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11011111;
  // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= 0b11100000;
}

// ADC Read Function
// Performs an ADC read on the specified channel.
unsigned int adc_read(unsigned char adc_channel_num) {
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;
  // clear the channel selection bits (MUX 5) hint: it's not in the ADMUX register
  *my_ADCSRB &= 0b11110111;
  // set the channel selection bits for channel 0
  *my_ADMUX += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register and format the data based on right justification (check the lecture slide)
  unsigned int val = (*my_ADC_DATA & 0x03FF);
  return val;
}

// UART Initialization Function
// Initializes the UART (Universal Asynchronous Receiver-Transmitter) for serial communication.
void U0init(int U0baud) {
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

// maybe not needed since using serial to print
// unsigned char U0kbhit(){
//   return *myUCSR0A & RDA;
// }
// unsigned char U0getchar(){
//   return *myUDR0;
// }
void U0putchar(unsigned char U0pdata){
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void FanON(bool on){
  // default state off
  analogWrite(enA, 255);
  if(on == true){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  if(on == false){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void RTCErrors(int e){
  if(e == 0){ // "Couldn't find RTC"
    U0putchar('C');
    U0putchar('o');
    U0putchar('u');
    U0putchar('l');
    U0putchar('d');
    U0putchar('n');
    U0putchar('\'');
    U0putchar('t');
    U0putchar(' ');
    U0putchar('f');
    U0putchar('i');
    U0putchar('n');
    U0putchar('d');
    U0putchar(' ');
    U0putchar('R');
    U0putchar('T');
    U0putchar('C');        
  }
  if(e == 1){ // "RTC is NOT running, setting the time"
    U0putchar('R');
    U0putchar('T');
    U0putchar('C');
    U0putchar(' ');
    U0putchar('i');
    U0putchar('s');
    U0putchar(' ');
    U0putchar('N');
    U0putchar('O');
    U0putchar('T');
    U0putchar(' ');
    U0putchar('r');
    U0putchar('u');
    U0putchar('n');
    U0putchar('n');
    U0putchar('i');
    U0putchar('n');
    U0putchar('g');
    U0putchar(',');
    U0putchar(' ');
    U0putchar('s');
    U0putchar('e');
    U0putchar('t');
    U0putchar('t');
    U0putchar('i');
    U0putchar('n');
    U0putchar('g');
    U0putchar(' ');
    U0putchar('t');
    U0putchar('h');
    U0putchar('e');
    U0putchar(' ');
    U0putchar('t');
    U0putchar('i');
    U0putchar('m');
    U0putchar('e');
  }
}

void SerialStates(int state){
  if(state == 1){
    char disabled[15] = {'D', 'i', 's', 'a', 'b', 'l', 'e', 'd', ' ', 'S', 't', 'a', 't', 'e', '\n'};
    for(int i = 0; i < 15; i++){
      U0putchar(disabled[i]);
    }
  }
  if(state == 2){
    char idle[11] = {'I', 'd', 'l', 'e', ' ', 'S', 't', 'a', 't', 'e', '\n'};
    for(int i = 0; i < 11; i++){
      U0putchar(idle[i]);
    }
  }
  if(state == 3){
    char running[14] = {'R', 'u', 'n', 'n', 'i', 'n', 'g', ' ', 'S', 't', 'a', 't', 'e', '\n'};
    for(int i = 0; i < 14; i++){
      U0putchar(running[i]);
    }
  }
  if(state == 4){
    char error[12] = {'E', 'r', 'r', 'o', 'r', ' ', 'S', 't', 'a', 't', 'e', '\n'};
    for(int i = 0; i < 12; i++){
      U0putchar(error[i]);
    }
  }
}