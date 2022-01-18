/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low magnetometer data registers.
They can be converted to units of gauss using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An LIS3MDL gives a magnetometer X axis reading of 1292 with its
default full scale setting of +/- 4 gauss. The GN specification
in the LIS3MDL datasheet (page 8) states a conversion factor of 6842
LSB/gauss (where LSB means least significant bit) at this FS setting, so the raw
reading of 1292 corresponds to 1292 / 6842 = 0.1888 gauss, 1 gauss = 100uT.

Attention : les niveaux logiques du LIS3MDL sont en 0 - 3.3V, ceux des arduino
Uno et Mega sont en 0 - 5V.
*/

#include <Wire.h>
#include <LIS3MDL.h>

#include <RunningMedian.h>

#define LED RED_LED

LIS3MDL mag1; 
LIS3MDL mag2;

unsigned long currentTime=0;
unsigned long previousTime=0;
unsigned long interval,interval1=200, interval2=2000;

bool ledState=LOW;


RunningMedian samples = RunningMedian(7);

float mes = 0;
char report[80];

void setup()
{
  Serial.begin(9600);
  //Serial.println("hello world");
  Wire.begin();

  while (!mag1.init(LIS3MDL::device_auto, LIS3MDL::sa1_high));//Mag plus proche de l'inductance
  while (!mag2.init(LIS3MDL::device_auto, LIS3MDL::sa1_low));
  
  mag1.enableDefault();
  mag1.writeReg(LIS3MDL::CTRL_REG1, 0x00);//X et Y en Low Power
  mag1.writeReg(LIS3MDL::CTRL_REG3, 0x01);//Configuration Low Power, Single conversion
  mag1.writeReg(LIS3MDL::CTRL_REG4, 0x0C);//Z Ultra High Perf (mesure du champ sur Z)
  mag1.writeReg(LIS3MDL::CTRL_REG5, 0x40);//Z Ultra High Perf (mesure du champ sur Z)

  mag2.enableDefault();
  mag2.writeReg(LIS3MDL::CTRL_REG1, 0x00);//X et Y en Low Power
  mag2.writeReg(LIS3MDL::CTRL_REG3, 0x01);//Configuration Low Power, Single conversion
  mag2.writeReg(LIS3MDL::CTRL_REG4, 0x0C);//Z Ultra High Perf (mesure du champ sur Z)
  mag2.writeReg(LIS3MDL::CTRL_REG5, 0x40);//Z Ultra High Perf (mesure du champ sur Z)

  pinMode(LED, OUTPUT);

}

void displaying_led_infos()
{
  int offset = 0;
  //Lecture des registres de sortie en I2C
  mag1.read();
  mag2.read();

  //Concatenation des resultats
  snprintf(report, sizeof(report),
  "M1: x: %6d y: %6d z: %6d\tM2: x: %6d y: %6d z: %6d",
    mag1.m.x, mag1.m.y, mag1.m.z, mag2.m.x, mag2.m.y, mag2.m.z);

  //Affichage en serial
  
  //Serial.println(report);


  //2005 : Correction d'offset
  //7.8 : 7.8mA/uT
  //68.42 : Passage de gauss a uT
  float r = 0.5;
  //mes = ((((2005+mag1.m.z)-mag2.m.z)/2)*7.1)/68.42;
  //mes = ((((mag1.m.z)-mag2.m.z)/2)*7.1)/68.42;
  //mes = (((((offset+mag1.m.z)-mag2.m.z)/2)*400) * 0.0005)/(2*pow(10,-7));
  mes = ((offset+mag1.m.z)-mag2.m.z)*r*0.036539;

  samples.add(mes);
  // Affichage de la moyenne
  Serial.print(mes);
  Serial.println(" mA");

  //Lancement de mesure
  mag1.writeReg(mag1.CTRL_REG3, 0x01);
  mag2.writeReg(mag1.CTRL_REG3, 0x01);
}

void loop(){

  

  currentTime=millis();
  
  if((currentTime-previousTime)>5000){
    
    previousTime=currentTime;
    
    ledState=!ledState;
    
    digitalWrite(LED,!ledState);
  }else if((currentTime-previousTime)%20 == 0){
    displaying_led_infos();
  }



}
