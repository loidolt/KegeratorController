/*
* Kegerator Controller V1
* Chris Loidolt
* 
* References:
* Temperature Sensor Displayed on 4 Digit 7 segment common CATHODE
* https://gist.github.com/ruisantos16/5419223
* 
* Refridgerator Thermostat
* https://github.com/rocketboy001/RefrigeratorThermostat/blob/master/ArduinoSketch/ArduinoSketch.ino
* 
*/
#include <DallasTemperature.h>
#include <OneWire.h>

// DS18B20 setup
#define ONE_WIRE_BUS 10
OneWire ds(ONE_WIRE_BUS);
DallasTemperature sensors(&ds);

int potValue = 0;
int currentTemp = 0;
int setTemp = 35;

int state = 2; // Initial state

bool CoolRun = false;
bool Boot = true;

unsigned long Sleep = 0;
unsigned long SleepLength = 900000; // How long we wait before we try to cool the fridge again.

int potPin = A0;
int relayOne = A5;
int relayTwo = A4;
int relayThree = A3;

// Display setup
const int digitPins[4] = {5,4,3,2}; //4 common CATHODE pins of the display (inverted the pins order)
const int clockPin = 6;    //74HC595
const int latchPin = 7;    //74HC595
const int dataPin = 8;     //74HC595
const byte digit[10] =      //seven segment digits in bits
{
  B00111111, //0
  B00000110, //1
  B01011011, //2
  B01001111, //3
  B01100110, //4
  B01101101, //5
  B01111101, //6
  B00000111, //7
  B01111111, //8
  B01101111  //9
};
int digitBuffer[4] = {0};
int digitScan = 0;


void setup(){
  delay(1000);

  pinMode(potPin, INPUT);
  
  pinMode(relayOne, OUTPUT);
  pinMode(relayTwo, OUTPUT);
  pinMode(relayThree, OUTPUT);

  for(int i=0;i<4;i++)
  {
    pinMode(digitPins[i],OUTPUT);
  }
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  //Serial.begin(9600);

  sensors.begin();
}

void loop(){

  switch(state) {
    case 1:
      active();
    break;
    
    case 2:
      sleeping();
    break;
  }
}

void active(){

  //Serial.println("Active");

  updateSensors();
  readPot();

  if (currentTemp > setTemp) { 
    digitalWrite(relayOne, HIGH); // Turn on the compressor
    //Serial.println("ON");
    CoolRun = true;
  }
  
  if (CoolRun == true && currentTemp <= setTemp) { 
    Sleep = millis();
    state = 2;
  }

  digitBuffer[3] = ((setTemp)%100)%10;
  digitBuffer[2] = (setTemp%100)/10;
  digitBuffer[1] = ((currentTemp)%100)%10;
  digitBuffer[0] = (currentTemp%100)/10;
  updateDisp();

}

void sleeping(){

  //Serial.println("Sleeping");

  updateSensors();
  readPot();

  digitalWrite(relayOne, LOW); // Shut off the compressor
  //Serial.println("OFF");
  
  if (millis() - Sleep > SleepLength) {
    CoolRun = false; 
    state = 1;
  }

  digitBuffer[3] = ((setTemp)%100)%10;
  digitBuffer[2] = (setTemp%100)/10;
  digitBuffer[1] = ((currentTemp)%100)%10;
  digitBuffer[0] = (currentTemp%100)/10;
  updateDisp();
  
}

void updateDisp(){ //writes the display
  
  for(byte j=0; j<4; j++) {digitalWrite(digitPins[j], HIGH);} // Turns the display off. Changed to HIGH
  digitalWrite(latchPin, LOW); 
  shiftOut(dataPin, clockPin, MSBFIRST, B00000000);
  digitalWrite(latchPin, HIGH);

  delayMicroseconds(2);

  digitalWrite(digitPins[digitScan], LOW); //Changed to LOW for turning the leds on.

  digitalWrite(latchPin, LOW); 

  if(digitScan==1)
    shiftOut(dataPin, clockPin, MSBFIRST, (digit[digitBuffer[digitScan]] | B10000000)); //print the decimal point on the 2nd digit
  else
    shiftOut(dataPin, clockPin, MSBFIRST, digit[digitBuffer[digitScan]]);

  digitalWrite(latchPin, HIGH);
  
  digitalWrite(latchPin, LOW);

  if(digitScan==3 && CoolRun==true)
    shiftOut(dataPin, clockPin, MSBFIRST, (digit[digitBuffer[digitScan]] | B10000000)); //print the decimal point on the 4th digit if cooling
  else
    shiftOut(dataPin, clockPin, MSBFIRST, digit[digitBuffer[digitScan]]);

  digitalWrite(latchPin, HIGH);

  digitScan++;
  if(digitScan>3) digitScan=0;
}

void updateSensors(){                                           // Check the sensors and update values

  // Check sensors every 30 seconds
  const unsigned long minutes = 30 * 1000UL;
  static unsigned long lastSampleTime = 0 - minutes;  // initialize such that a reading is due the first time through loop()
  
  unsigned long now = millis();
  if (now - lastSampleTime >= minutes)
  {
    lastSampleTime += minutes;
    sensors.requestTemperatures();
    currentTemp = sensors.getTempFByIndex(0);
    //Serial.println(currentTemp);
  }
}

void readPot(){
  
  potValue = analogRead(potPin);    // read the value from the sensor
  //Serial.println(potValue);
  setTemp = 30 + (potValue / 40);
  //Serial.println(setTemp);
  
}

