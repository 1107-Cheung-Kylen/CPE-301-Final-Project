// Authors: Kylen Cheung and Jonathan Summers

// Button - A15 (PK7)
// Opens analog for ports Port K0-7
volatile unsigned char* port_k = (unsigned char*) 0x108; 
volatile unsigned char* ddr_k  = (unsigned char*) 0x107; 
volatile unsigned char* pin_k  = (unsigned char*) 0x106; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // set PK2 to input
  *ddr_k &= 0b10000000;  // 
  // set PD0 to output
}

void loop() {
  // put your main code here, to run repeatedly:
  if(*pin_k & 0b10000000){ // if high;
    Serial.println("High");
  }
  else{ // if low
    Serial.println("Low");
  }
  delay(100);
}
