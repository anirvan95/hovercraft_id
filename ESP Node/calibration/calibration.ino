//Calibration Sketch
#include <ESP32_Servo.h>

#define ESC_CAL_DELAY  2900  // Calibration delay (milisecond)
#define ESC_STOP_PULSE  500 //



Servo betaTL, betaTR, betaFL, betaFR, betaBL, betaBR;  // create servo object to control a servo

int escPinTL = 19;
int escPinTR = 21;

int escPinFL = 22;
int escPinFR = 23;

int escPinBL = 27;
int escPinBR = 12;

int oESC;
int flag = 1;
void setup() {
  pinMode(13, OUTPUT);  
  betaTL.attach(escPinTL);   
  betaTR.attach(escPinTR);                            
  betaFL.attach(escPinFL);                            
  betaFR.attach(escPinFR);                          
  betaBL.attach(escPinBL);                           
  betaBR.attach(escPinBR);                             // using default min/max of 1000us and 2000us
  
  betaTL.writeMicroseconds(2000);
  betaTR.writeMicroseconds(2000);
  betaFL.writeMicroseconds(2000);
  betaFR.writeMicroseconds(2000);
  betaBL.writeMicroseconds(2000);
  betaBR.writeMicroseconds(2000);
  delay(ESC_CAL_DELAY);
  betaTL.writeMicroseconds(1000);
  betaTR.writeMicroseconds(1000);
  betaFL.writeMicroseconds(1000);
  betaFR.writeMicroseconds(1000);
  betaBL.writeMicroseconds(1000);
  betaBR.writeMicroseconds(1000);
  
  delay(ESC_CAL_DELAY);
  delay(5000);
  
  digitalWrite(13, HIGH);
  Serial.begin(9600);
}

void loop() 
{
 
  Serial.print("Calibration completed");


}
