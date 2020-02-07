#include <SPI.h>

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600); 

pinMode(PB_0,OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly: 
  sendSPI(0b111100000011, PB_0);
}

void sendSPI(int data, byte cspin){
  int conf = 0b01110000;
  int dummy = (data & 0b0000111100000000)>>8;
  byte byte1 = conf | dummy;
  byte byte2 = (byte)((data & 0b0000000011111111));
  //Serial.print("\t output = "); 
  //Serial.println(byte1);  
  
  }
