// Authors: Kylen Cheung and Jonathan Summers

// Button - PG0
volatile unsigned char* port_g = (unsigned char*) 0x34; 
volatile unsigned char* ddr_g  = (unsigned char*) 0x33; 
volatile unsigned char* pin_g  = (unsigned char*) 0x32; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // set PK2 to input
  *ddr_g &= 0b00000001;  // 
  // set PD0 to output
}

void loop() {
  // put your main code here, to run repeatedly:
  if(*pin_g & 0b00000001){ // if high;
    Serial.println("High");
  }
  else{ // if low
    Serial.println("Low");
  }
  delay(100);
}
