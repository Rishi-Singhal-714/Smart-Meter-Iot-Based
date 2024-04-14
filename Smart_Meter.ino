#include <Wire.h>
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>


#define OLED_ADDRESS 0x3C
#define OLED_SDA A4
#define OLED_SCL A5
#define light_ON    (0)
#define light_OFF   (1)
#define Fan_ON    (2)
#define Fan_OFF   (3)

const int ACS712_PIN = A2;

// Calibration values obtained from the sketch: volt_ac_cal
int adc_max = 760;                // Maximum sensor value during calibration
int adc_min = 261;                // Minimum sensor value during calibration

float volt_multi = 231;           // RMS voltage obtained from a multimeter
float volt_multi_p;               // Peak voltage
float volt_multi_n;               // Negative peak voltage

// ZMPT101B Configuration
const int ZMPT101B_PIN = A3;

VR myVR(2, 3);   // 2:RX 3:TX, you can choose your favourite pins.

uint8_t records[7]; // save record
uint8_t buf[64];

/* define L298N motor drive control pins */
int Light = A0;    // IN1
int Fan = A1;   // IN2





void printSignature(uint8_t *buf, int len)
{
  int i;
  for (i = 0; i < len; i++) {
    if (buf[i] > 0x19 && buf[i] < 0x7F) {
      Serial.write(buf[i]);
    }
    else {
     // Serial.print("[");
      //Serial.print(buf[i], HEX);
      //Serial.print("]");
    }
  }
}


void printVR(uint8_t *buf)
{
  //Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  //Serial.print(buf[2], DEC);
  //Serial.print("\t\t");

  if (buf[0] == 0xFF) {
    //Serial.print("NONE");
  }
  else if (buf[0] & 0x80) {
    //Serial.print("UG ");
    //Serial.print(buf[0] & (~0x80), DEC);
  }
  else {
    //Serial.print("SG ");
    //Serial.print(buf[0], DEC);
  }
  //Serial.print("\t");

  //Serial.print(buf[1], DEC);
  //Serial.print("\t\t");
  if (buf[3] > 0) {
    printSignature(buf + 4, buf[3]);
  }
  else {
    //Serial.print("NONE");
  }
  //Serial.println("\r\n");
}
Adafruit_SSD1306 display(128, 64, &Wire, OLED_ADDRESS);
void setup()
{
   Wire.begin();
  /** initialize */
  myVR.begin(9600);

  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Smart Home");
  display.display();
  
  //Serial.println("Voice Control start");

  /* initialize motor control pins as output */
  pinMode(Light, OUTPUT);
  pinMode(Fan, OUTPUT);

  digitalWrite(Light, HIGH);
  digitalWrite(Fan, HIGH);

  if (myVR.clear() == 0) {
   // Serial.println("Recognizer cleared.");
  } else {
   // Serial.println("Not find VoiceRecognitionModule.");
   // Serial.println("Please check connection and restart Arduino.");
    while (1);
  }

  if (myVR.load((uint8_t)light_ON) >= 0) {
   // Serial.println("Light ON loaded");
  }

  if (myVR.load((uint8_t)light_OFF) >= 0) {
    //Serial.println("Light OFF Loaded");
  }
  if (myVR.load((uint8_t)Fan_ON) >= 0) {
   // Serial.println("Fan ON loaded");
  }

  if (myVR.load((uint8_t)Fan_OFF) >= 0) {
    //Serial.println("Fan OFF loaded");
  }

  delay(1000);   // Add a delay to allow time for OLED initialization
  //Serial.println("Voice Control start");
  volt_multi_p = volt_multi * 1.4142;   // Peak voltage = RMS voltage * 1.4142 (Single-phase current)
  volt_multi_n = -volt_multi_p;         // Negative peak voltage
}

void loop()
{
  // ACS712 Code
  float acs_current = getACSCurrent();  // ACS712 current measurement

  // ZMPT101B Code
  float zmpt_voltage = getZMPTVoltage();// ZMPT101B voltage measurement
  
  
  int ret;
  ret = myVR.recognize(buf, 50);
  if (ret > 0) {
    switch (buf[1]) {
      case light_ON:
        digitalWrite(Light, LOW);
        break;

      case light_OFF:
        digitalWrite(Light, HIGH);
        break;

      case Fan_ON:
        digitalWrite(Fan, LOW);
        break;

      case Fan_OFF:
        digitalWrite(Fan, HIGH);
        break;

      default:
        Serial.println("Record function undefined");
        break;
    }
    /** voice recognized */
    printVR(buf);
  }
 
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Current: ");
  display.print(acs_current);
  display.println(" A");
  
  display.setCursor(0, 20);
  display.print("Voltage: ");
  display.print(zmpt_voltage);
  display.println(" V");
  display.display();
  delay(1000);
}
float getACSCurrent() {
  unsigned int x = 0;
  float AcsValue = 0.0, Samples = 0.0, AvgAcs = 0.0, AcsValueF = 0.0;

  for (int x = 0; x < 150; x++) { // Get 150 samples
    AcsValue = analogRead(ACS712_PIN);   // Read ACS712 sensor values
    Samples = Samples + AcsValue;        // Add samples together
    delay(3); // Let ADC settle before next sample 3ms
  }

  AvgAcs = Samples / 150.0; // Taking Average of Samples

  // ((AvgAcs * (5.0 / 1024.0)) is converting the read voltage in 0-5 volts
  // 2.5 is offset (I assumed that Arduino is working on 5v, so the Vout at no current comes
  // out to be 2.5 which is our offset. If your Arduino is working on a different voltage, then
  // you must change the offset according to the input voltage)
  // 0.100v (100mV) is the rise in output voltage when 1A current flows at input
  AcsValueF = (2.5 - (AvgAcs * (5.0 / 1024.0))) / 0.100;

  return AcsValueF;
}

float getZMPTVoltage() {
  float adc_sample;
  float volt_inst = 0;
  float sum = 0;
  float volt;
  long init_time = millis();
  int N = 0;

  while ((millis() - init_time) < 500) {   // Duration of 0.5 seconds (Approximately 30 cycles of 60Hz)
    adc_sample = analogRead(ZMPT101B_PIN); // Sensor voltage
    volt_inst = map(adc_sample, adc_min, adc_max, volt_multi_n, volt_multi_p);
    sum += sq(volt_inst);                    // Sum of Squares
    N++;
    delay(1);
  }

  volt = sqrt(sum / N);                     // RMS equation
  return volt;
}
