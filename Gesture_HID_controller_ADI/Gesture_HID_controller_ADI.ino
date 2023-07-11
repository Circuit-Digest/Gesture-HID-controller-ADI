#define US_KEYBOARD 1

#include <Arduino.h>
#include <Preferences.h>
#include <BleKeyboard.h>
#include <Wire.h>
#define R2 105
#define R3 105
#define VOLTAGE_OUT(Vin) (((Vin)*R3) / (R2 + R3))
#define VOLTAGE_MAX 4200
#define VOLTAGE_MIN 3300
#define ADC_REFERENCE 1100
#define VOLTAGE_TO_ADC(in) ((ADC_REFERENCE * (in)) / 4096)
#define ADC_TO_VOLTAGE(in1) (((in1 * 3300) / 4096) * 2)
#define BATTERY_MAX_ADC VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MAX))
#define BATTERY_MIN_ADC VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MIN))
#define BATTERY_volt VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MIN))
#define DEVICE (0x53)  // ADXL345 I2C address (fixed)

byte _buff[6];

char POWER_CTL = 0x2D;

char DATA_FORMAT = 0x31;

char DATAX0 = 0x32;  // x-axis data register byte 0
char DATAX1 = 0x33;  // x-axis data register byte 1
char DATAY0 = 0x34;  // y-axis
char DATAY1 = 0x35;  //
char DATAZ0 = 0x36;
char DATAZ1 = 0x37;

// Averaged readings from 2-point calibration with 6-point-tumble method (values particular to each IC)
const int16_t xUp = 254;     // Delta -2 to 256
const int16_t xDown = -262;  // Delta +6
const int16_t yUp = 257;     // Delta +1
const int16_t yDown = -255;  // Delta -1
const int16_t zUp = 255;     // Delta -1
const int16_t zDown = -242;  // Delta -14

float xOffset, xGain, yOffset, yGain, zOffset, zGain;

const float alphaEMA = 0.2;  // Smoothing factor 0 < α < 1 (smaller = smoother = less responsive)
float xEMA = 0;              // Seed with an arbitrary reading (0 = PCB oriented z-up at start)
float yEMA = 0;
float zEMA = 256;

BleKeyboard bleKeyboard("ADXL Controller", "Circuit Digest");
int current_s, current_w;
int bat_p;
int dutyCycle = 0, fade = 0;
uint64_t last_blink = 0, last_fade = 0;
// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.println("Btkeyboard");
  // make the pushbutton's pin an input:
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(3, INPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  digitalWrite(13, 0);
  digitalWrite(14, 0);
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(14, ledChannel);
  Wire.begin(38, 39);
  registerWrite(DATA_FORMAT, 0x00);  // Set to 2g mode, typical output -256 +256 per axis, see p. 4 http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
  registerWrite(POWER_CTL, 0x08);    // Set to measuring mode

  // Cast ints to float at execution time, as per Analog Devices and Sparkfun tutorial
  xOffset = 0.5 * ((float)xUp + (float)xDown);        // -4
  xGain = 0.5 * (((float)xUp - (float)xDown) / 256);  // 1.008
  yOffset = 0.5 * ((float)yUp + (float)yDown);        // +1
  yGain = 0.5 * (((float)yUp - (float)yDown) / 256);  // 1
  zOffset = 0.5 * ((float)zUp + (float)zDown);        // +6.5
  zGain = 0.5 * (((float)zUp - (float)zDown) / 256);  // 0.971
  bleKeyboard.begin();
  bat_p = calc_battery_percentage(analogRead(3));
  bleKeyboard.setBatteryLevel(bat_p);
}

// the loop routine runs over and over again forever:
void loop() {
  int temp = calc_battery_percentage(analogRead(3));
  if (bat_p != temp) {
    bat_p = temp;
    bleKeyboard.setBatteryLevel(bat_p);
  }
  if (bleKeyboard.isConnected()) {
    digitalWrite(13, HIGH);
    readSensor();
    current_s = digitalRead(6);
    current_w = digitalRead(7);
    if (current_s == 0) {
      bleKeyboard.write('S');
      delay(10);
      Serial.println("S key pressed");
    }
    if (current_w == 0) {
      bleKeyboard.write('W');
      delay(10);
      Serial.println("W key pressed");
    }
    if (yEMA > 150) {
      bleKeyboard.press('D');
      Serial.println("D key pressed");
      delay(50);
      bleKeyboard.releaseAll();
      Serial.println("D key released");
    }
    if (yEMA < -150) {
      bleKeyboard.press('A');
      Serial.println("A key pressed");
      delay(50);
      bleKeyboard.releaseAll();
      Serial.println("A key released");
    }
  } else if (millis() - last_blink > 300) {
    digitalWrite(13, !digitalRead(13));
    last_blink = millis();
  }
  if (bat_p < 25) {
    if (millis() - last_fade > 10) {
      if (fade == 0) {
        dutyCycle++;
        if (dutyCycle > 150) {
          fade = 1;
        }
      }
      else{
        dutyCycle--;
        if (dutyCycle < 1) {
          fade = 0;
        }
      }
      ledcWrite(ledChannel, dutyCycle);
    }
  } else {
    ledcWrite(ledChannel, 0);
  }
  delay(10);  // delay in between reads for stability
}

void readSensor() {                          // Read from the sensor, calibrate readings, smooth output
  uint8_t bytesToRead = 6;                   // Burst read (preferential as per Analog Devices)
  registerRead(DATAX0, bytesToRead, _buff);  // Read from the 6 registers

  int16_t xRaw = (((int)_buff[1]) << 8) | _buff[0];  // 10 bit (2 bytes), LSB first, convert into integer
  int16_t yRaw = (((int)_buff[3]) << 8) | _buff[2];
  int16_t zRaw = (((int)_buff[5]) << 8) | _buff[4];

  float xCal = ((float)xRaw - xOffset) / xGain;  // See p. 8 http://www.analog.com/media/en/technical-documentation/application-notes/AN-1057.pdf
  float yCal = ((float)yRaw - yOffset) / yGain;
  float zCal = ((float)zRaw - zOffset) / zGain;

  xEMA = (alphaEMA * xCal) + ((1 - alphaEMA) * xEMA);  // See https://en.wikipedia.org/wiki/Exponential_smoothing
  yEMA = (alphaEMA * yCal) + ((1 - alphaEMA) * yEMA);
  zEMA = (alphaEMA * zCal) + ((1 - alphaEMA) * zEMA);

  //Serial.print(xEMA,0);
  //Serial.print(", ");
  //Serial.print(yEMA,0);
  //Serial.print(", ");
  //Serial.println(zEMA,0);
}

void registerWrite(byte address, byte val) {
  Wire.beginTransmission(DEVICE);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

void registerRead(byte address, byte num, byte _buff[])  // Reads num bytes into _buff array
{
  Wire.beginTransmission(DEVICE);
  Wire.write(address);
  Wire.endTransmission();
  //Wire.beginTransmission(DEVICE);
  Wire.requestFrom(DEVICE, num);

  byte i = 0;

  while (Wire.available()) {
    _buff[i] = Wire.read();  // Read 1 byte
    i++;
  }

  Wire.endTransmission();
}
int calc_battery_percentage(int adc) {
  int battery_percentage = 100 * (adc - BATTERY_MIN_ADC) / (BATTERY_MAX_ADC - BATTERY_MIN_ADC);

  if (battery_percentage < 0)
    battery_percentage = 0;
  if (battery_percentage > 100)
    battery_percentage = 100;

  return battery_percentage;
}