// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation #include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"     // class default I2C address is 0x68 
#include <PID_v1.h>
MPU6050 accelgyroIC1(0x68);  // Here onwards it is different in state-space method
byte countS = 0;                        //05
int recOmegaI[10];                  
int omegaI = 0;                         
long thetaI = 0;                        
long sumPower = 0;                    
long sumSumP = 0;             //10
const int kAngle = 50;        
const int kOmega = 600;   
const long kSpeed = 60;   
const long kDistance = 20;  
long powerScale;    //15
int power;      
long vE5 = 0;       
long xE5 = 0;       
int16_t ax1, ay1, az1; 
int16_t gx1, gy1, gz1;    //20
float anv1;   
#define LED_PIN 13 
bool blinkState = false; 
void setup() 
{       //25
   Wire.begin();                                              
   Serial.begin(230400); 
   Serial.println("Initializing I2C devices..."); 
   accelgyroIC1.initialize();
   pinMode(LED_PIN, OUTPUT);   //30
   pinMode(7, OUTPUT);
   pinMode(8, OUTPUT);
   pinMode(9, OUTPUT);
} 
void loop()         //35
{ 
// read raw accel/gyro measurements from device 
 accelgyroIC1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);       
 anv1=(float(gx1))/131;  //getting instantaneous angular velocity
                                  //200 was the gyroscope offset value for me
                                 //131 is the LBS sensitivity
  chkAndCtl();
 if ( power > 0 ) 
  { 
   analogWrite( 9, power );   //45
   digitalWrite( 7, HIGH );
   digitalWrite( 8, LOW );
   }
   else {
     analogWrite( 9, - power );   //50
     digitalWrite( 7, LOW );
     digitalWrite( 8, HIGH );
   } 
  delay(5);                                  
 }          //55
 
 void chkAndCtl() { 
   omegaI = anv1; 
   if ( abs( omegaI ) < 2 ) { omegaI = 0; }        //58  (The lower bound is less than 2)
   recOmegaI[0] = omegaI;
   thetaI = thetaI + omegaI;
   countS = 0; 
   
   for ( int i = 0 ; i < 10 ; i++ ) 
    { 
     if ( abs( recOmegaI[i] ) < 4 ) { countS++; }    //64 (The lower bound is less than 4)
     } 
  
  if ( countS > 9 ) { //this thing just corrects if the 0 point, i.e. the angle at which CG is exactly above point of contact
     thetaI = 0;            
     //vE5 = 0;
     //xE5 = 0;
     sumPower = 0;
     sumSumP = 0;          //70
     }              
  
  //All the K parameters below represent the elements of feedback control K matrix.
   for ( int i = 9 ; i > 0 ; i-- ) { recOmegaI[ i ] = recOmegaI[ i-1 ]; }
  
   powerScale = ( kAngle * thetaI / 100 ) + ( kOmega * omegaI / 100 ) + ( kSpeed * vE5 / 1000 ) + ( kDistance * xE5 / 1000 ) ; 
   power = max ( min ( 95 * powerScale / 100 , 255 ) , -255 );
   sumPower = sumPower + power;
   sumSumP = sumSumP + sumPower;    
   vE5 = sumPower;         
   xE5 = sumSumP / 1000; 
   }
