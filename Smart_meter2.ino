#include "EmonLib.h"
#include <EEPROM.h>

EnergyMonitor emon;
#define vCalibration 83.3
#define currCalibration 0.50

float kWh = 0;
unsigned long lastmillis = millis();
unsigned long interval = 5000;  // Time in milliseconds
unsigned long lastInterval = 0;

void myTimerEvent()
{
  emon.calcVI(20, 2000);
  kWh = kWh + emon.apparentPower * (millis() - lastmillis) / 3600000000.0;
  yield();
  Serial.print("Vrms: ");
  Serial.print(emon.Vrms, 2);
  Serial.print("V");
  EEPROM.put(0, emon.Vrms);
  delay(100);

  Serial.print("\tIrms: ");
  Serial.print(emon.Irms , 4);  // Add 2.3 A to the measured Irms
  Serial.print("A");
  EEPROM.put(4, emon.Irms);
  delay(100);

  Serial.print("\tPower: ");
  Serial.print(emon.apparentPower, 4);
  Serial.print("W");
  EEPROM.put(8, emon.apparentPower);
  delay(100);

  Serial.print("\tkWh: ");
  Serial.print(kWh, 5);
  Serial.println("kWh");
  EEPROM.put(12, kWh);

  lastmillis = millis();
}

void setup()
{
  Serial.begin(115200);
  emon.voltage(35, vCalibration, 1.9);  // Voltage: input pin, calibration, phase_shift
  emon.current(34, currCalibration);    // Current: input pin, calibration.
  
  lastInterval = millis(); // Initialize last interval
  
  Serial.println("Initializing...");
}

void loop()
{
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastInterval >= interval) {
    lastInterval = currentMillis;
    myTimerEvent(); // Call the function
  }
}
