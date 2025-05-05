#include <LiquidCrystal.h> 
#include "DHT.h" // DHT Sensor Library by Adafruit
#include <Stepper.h>
#include <Wire.h>
#include "RTClib.h" // RTClib by Adafruit

// Environmental threshold variables
float tempThreshold = 78;  // Temperature threshold
float waterThreshold = 500; // Water level threshold

// Stepper motor configuration
const int stepsPerRevolution = 2048;  // Number of steps for a full revolution
const int rpm = 10;  // Rotations per minute
Stepper stepper(stepsPerRevolution, 15, 17, 16, 18);  // Initialize the stepper library on pins 8 through 11

// DHT sensor setup
#define DHTPIN 14  // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor

// LCD display setup
LiquidCrystal lcd(11, 12, 2, 3, 4, 5);  // Set the LCD pins

// Clock variables
RTC_DS1307 rtc;  // Real Time Clock object
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Fan control pins
#define ENABLE 22
#define DIRA 23
#define DIRB 25

// ADC (Analog-to-Digital Converter) setup // NOT USED?
#define RDA 0x80
#define TBE 0x20 

// Water sensor
volatile unsigned char *adcMux = (unsigned char*) 0x7C;
volatile unsigned char *adcControlStatusA = (unsigned char*) 0x7A;
volatile unsigned char *adcControlStatusB = (unsigned char *) 0x7b;
volatile unsigned int *adcData = (unsigned int*) 0x78;

volatile unsigned char *serialControlRegA = (unsigned char *)0x00C0;
volatile unsigned char *serialControlRegB = (unsigned char *)0x00C1;
volatile unsigned char *serialControlRegC = (unsigned char *)0x00C2;
volatile unsigned int  *baudRateReg  = (unsigned int *) 0x00C4;
volatile unsigned char *dataReg   = (unsigned char *)0x00C6;

volatile unsigned char *timerControlRegC = (unsigned char *) 0x82;
volatile unsigned char *timerInterruptMask1 = (unsigned char *) 0x6F;

long value = 0UL;
long maxtime = 160000000UL;

// Port K registers (Analog I/O pins)
volatile unsigned char* portKInput = (unsigned char*) 0x106;
volatile unsigned char* portKDirection = (unsigned char*) 0x107;
volatile unsigned char* portKData = (unsigned char*) 0x108;

// Timer configuration
volatile unsigned char* timerControlRegA = (unsigned char*) 0x80;
volatile unsigned char* timerControlRegB = (unsigned char*) 0x81;
volatile unsigned int* timerCounter1 = (unsigned int*) 0x84;
volatile unsigned char* timerInterruptFlagReg1 = (unsigned char*) 0x36;
int oneSecondTicks = 62500;  // Ticks value for 1 second delay with timer

// Port B registers (Digital I/O pins)
volatile unsigned char* portBData = (unsigned char*) 0x25; // Port B data register
volatile unsigned char* portBDirection = (unsigned char*) 0x24;  // Port B Data Direction Register
volatile unsigned char* portBInput = (unsigned char*) 0x23;  // Port B Input Pin Address

int toggle = 0;

// Function declarations
void adc_init();
unsigned int adc_read(unsigned char adc_channel);
void MyDelay(unsigned int ticks);
void U0putchar(int U0pdata);
void U0init(unsigned long U0baud);
int WaterSensor();
void TimerDelay(unsigned int ticks); 
void ClockModule(); 
void FanControl();
void VentControl();
void LCDError();
void LCDData(float h, float f);
double DHTSensor();
void ErrorState();
void RunningState();
void IdleState();
void DisabledState();

void setup() {
  // Initialize serial communication and RTC (Real Time Clock)
  // Serial.begin(57600);
  // #ifndef ESP8266
  // while (!Serial); // Wait for serial port to connect. Needed for native USB
  // #endif

  Serial.begin(9600);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);  // Infinite loop if RTC not found
  }

  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running, setting the time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Set RTC to compile time
  }

  // Initialize stepper motor
  // stepper.setSpeed(rpm);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("test");


  // TESTING
  dht.begin();

  // Initialize timers
  *timerInterruptFlagReg1 = B00000000;
  *timerCounter1 = B00000000;
  *timerControlRegB = B00000000;
  *timerControlRegA = B00000000;

  // Initialize ADC and UART
  U0init(9600);
  adc_init();

  // Initialize digital output pins
  *portBDirection = B10001111;

  // Initialize analog input pins
  *portKDirection = B01111111;
}

void loop() {
  // Main program loop
  // Check and handle system states based on sensor inputs and toggle switch

  //TESTING
  VentControl();

  // Disabled state: system off
  if (*portKInput == B00000001) {
    DisabledState();
  }

  // Idle state: system on, but idle
  if (*portKInput == B00000000 && (WaterSensor() > waterThreshold) && (DHTSensor() < tempThreshold)) {
    IdleState();
  }

  // Running state: active ventilation based on temperature and water level
  if (*portKInput == B00000000 && (WaterSensor() > waterThreshold) && (DHTSensor() > tempThreshold)) {
    RunningState();
  }

  // Error state: water level below threshold
  if (*portKInput == B00000000 && (WaterSensor() < waterThreshold)) {
    // ErrorState(); // disabled for TESTING
  }
}

// Disabled State Function
// This state represents the system being turned off or disabled.
void DisabledState() {
  if (toggle != 1) {
    toggle = 1;
    lcd.clear();
    *portBData &= B01111111;
    *portBData |= B00000010;
    *portBData &= B11111110;
    *portBData &= B11111011;
    *portBData &= B11110111;
  }
  Serial.println("DISABLED State");
  ClockModule();  // Display clock information
  *portBData |= B00000010;  // Turn on Yellow LED to indicate disabled state
  lcd.setCursor(0, 0);
  lcd.print("System Off");
}

// Idle State Function
// This state is active when the system is on but idle due to no need for intervention.
void IdleState() {
  if (toggle != 2) {
    toggle = 2;
    lcd.clear();
    *portBData &= B01111111;
    *portBData |= B00000001;
    *portBData &= B11111101;
    *portBData &= B11111011;
    *portBData &= B11110111;
  }
  Serial.println("IDLE State");
  ClockModule();
  DHTSensor();  // Read humidity and temperature
  VentControl();  // Control ventilation based on environmental data
}

// Running State Function
// This state is active when the system needs to run due to environmental conditions.
void RunningState() {
  if (toggle != 3) {
    toggle = 3;
    lcd.clear();
    *portBData |= B00001000;
    *portBData &= B11111101;
    *portBData &= B11111011;
    *portBData &= B11111110;
    *portBData |= B10000000;  // Turn on the fan
  }
  DHTSensor();
  VentControl();
  Serial.println("RUNNING State");
  ClockModule();
}

// Error State Function
// This state is triggered when there is an error, such as low water level.
void ErrorState() {
  if (toggle != 4) {
    toggle = 4;
    lcd.clear();
    *portBData &= B01111111;
    *portBData |= B00000100;
    *portBData &= B11110111;
    *portBData &= B11111101;
    *portBData &= B11111110;
  }
  VentControl();
  Serial.println("ERROR State");
  ClockModule();
  LCDError();  // Display error message on LCD
}

// DHT Sensor Function
// Reads and returns temperature from the DHT sensor, and updates the LCD.
double DHTSensor() {
  // float humidity = dht.readHumidity();  // Read humidity
  float humidity = dht.readHumidity();
  float temperatureF = dht.readTemperature(true);

  //TESTING
  Serial.println("TEMPERATURE");
  Serial.println(temperatureF);
  Serial.println("HUMIDITY");
  Serial.println(humidity);

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
  // int steps = stepsPerRevolution / 360;  // Calculate steps for partial revolution

  // while (*portKInput == B00000010) {  // Analog pin 9
  //   stepper.step(steps);  // Rotate stepper motor
  //   Serial.println("Rotating Left");
  // }

  // while (*portKInput == B00000100) {  // Analog pin 10
  //   stepper.step(-steps);  // Rotate stepper motor in opposite direction
  //   Serial.println("Rotating Right");
  // }

  stepper.setSpeed(5);
  stepper.step(stepsPerRevolution);
  delay(1000);
}

// Clock Module Function
// Displays the current date and time on the serial monitor.
void ClockModule() {
  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
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

// Water Sensor Function
// Reads and returns the water level from the ADC.
int WaterSensor() {
  adc_init();  // Initialize ADC
  int waterLevel = adc_read(0x00);  // Read water level from ADC channel 0
  return waterLevel;  // Return the water level reading
}

// ADC Initialization Function
// Sets up the ADC for reading.
void adc_init() {
  *adcMux |= 0b11000000;  // Set ADC multiplexer settings
  *adcControlStatusA |= 0b10100000;  // Set ADC control and status register A
  *adcControlStatusB |= 0b01000000;  // Set ADC control and status register B
  *adcData |= 0x00;  // Clear ADC data register
}

// ADC Read Function
// Performs an ADC read on the specified channel.
unsigned int adc_read(unsigned char adc_channel) {
  *adcControlStatusB |= adc_channel;  // Set channel to read from
  *adcControlStatusA |= 0x40;  // Start the ADC conversion
  while (!(*adcControlStatusA & 0x40));  // Wait for conversion to complete
  return *adcData;  // Return the ADC data
}

// UART Initialization Function
// Initializes the UART (Universal Asynchronous Receiver-Transmitter) for serial communication.
void U0init(unsigned long baudRate) {
  unsigned long fCPU = 16000000;  // CPU clock frequency
  unsigned int tBaud = (fCPU / 16 / baudRate - 1);  // Calculate baud rate
  *serialControlRegA = 0x20;  // Set control register A
  *serialControlRegB = 0x18;  // Set control register B
  *serialControlRegC = 0x06;  // Set control register C
  *baudRateReg = tBaud;  // Set baud rate register
}

