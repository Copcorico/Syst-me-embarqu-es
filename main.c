#include <ChainableLED.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>

// === PINS ===
#define PIN_LED_DATA 5
#define PIN_LED_CLK 6
#define PIN_LED_CNT 1
#define RED_BUTTON 2
#define GREEN_BUTTON 3
#define SD_CS 4
#define LIGHT_SENSOR A0

// === HARDWARE OBJECTS ===
BME280I2C bme;
RTC_DS3231 rtc;
ChainableLED leds(PIN_LED_DATA, PIN_LED_CLK, PIN_LED_CNT);
SoftwareSerial SoftSerial(8, 9); // GPS

// === EEPROM ADDRESSES ===
#define EEPROM_LOG_INTERVAL_ADDR 0
#define EEPROM_FILE_MAX_SIZE_ADDR 1
#define EEPROM_TIMEOUT_ADDR 3
#define EEPROM_LUMIN_ADDR 4
#define EEPROM_LUMIN_LOW_ADDR 5
#define EEPROM_LUMIN_HIGH_ADDR 7
#define EEPROM_TEMP_AIR_ADDR 9
#define EEPROM_MIN_TEMP_AIR_ADDR 10
#define EEPROM_MAX_TEMP_AIR_ADDR 12
#define EEPROM_HYGR_ADDR 14
#define EEPROM_HYGR_MINT_ADDR 15
#define EEPROM_HYGR_MAXT_ADDR 17
#define EEPROM_PRESSURE_ADDR 19
#define EEPROM_PRESSURE_MIN_ADDR 20
#define EEPROM_PRESSURE_MAX_ADDR 22

// === DEFAULTS ===
const uint8_t DEF_LOG_INTERVAL = 10;
const uint16_t DEF_FILE_MAX_SIZE = 2048;
const uint8_t DEF_TIMEOUT = 30;
const uint8_t DEF_LUMIN = 1;
const uint16_t DEF_LUMIN_LOW = 255;
const uint16_t DEF_LUMIN_HIGH = 768;
const uint8_t DEF_TEMP_AIR = 1;
const int16_t DEF_MIN_TEMP_AIR = -10;
const int16_t DEF_MAX_TEMP_AIR = 60;
const uint8_t DEF_HYGR = 1;
const int16_t DEF_HYGR_MINT = 0;
const int16_t DEF_HYGR_MAXT = 50;
const uint8_t DEF_PRESSURE = 1;
const int16_t DEF_PRESSURE_MIN = 850;
const int16_t DEF_PRESSURE_MAX = 1030;

// === GLOBAL CONFIG ===
uint8_t LOG_INTERVAL;
uint16_t FILE_MAX_SIZE;
uint8_t TIMEOUT;
uint8_t LUMIN;
uint16_t LUMIN_LOW;
uint16_t LUMIN_HIGH;
uint8_t TEMP_AIR;
int16_t MIN_TEMP_AIR;
int16_t MAX_TEMP_AIR;
uint8_t HYGR;
int16_t HYGR_MINT;
int16_t HYGR_MAXT;
uint8_t PRESSURE;
int16_t PRESSURE_MIN;
int16_t PRESSURE_MAX;

// === MODES ===
uint8_t modeActuel = 1;
uint8_t modePrecedent = 1;
unsigned long redPressStart = 0;
unsigned long greenPressStart = 0;
unsigned long lastConfigActivity = 0;
unsigned long lastMeasureTime = 0;
bool eco_gps_skip = false;

// SD logging
File currentFile;
char baseFilename[20];
void applyFilenameForNow(uint8_t revisionZero) {}
// Interrupt flags
volatile bool redPressed = false;
volatile bool greenPressed = false;
volatile unsigned long redPressTime = 0;
volatile unsigned long greenPressTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  rtc.begin();
  SoftSerial.begin(9600);
  pinMode(RED_BUTTON, INPUT_PULLUP);
  pinMode(GREEN_BUTTON, INPUT_PULLUP);
  leds.setColorRGB(0, 0, 255, 0);
  bme.begin();
  bool status = bme.begin();
  if (!status) {
    Serial.println(F("Erreur : capteur BME280 introuvable !"));
    indicateError(3);  
  }
  loadConfigFromEEPROM();

  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD init failed!"));
    while (true) {
      leds.setColorRGB(0, 255, 0, 0);
      delay(500);
      leds.setColorRGB(0, 255, 255, 255);
      delay(500);
    }
  }

  if (digitalRead(RED_BUTTON) == LOW) {
    modeActuel = 2;
    setLEDForMode(2);
    lastConfigActivity = millis();
  }

  applyFilenameForNow(0);

  attachInterrupt(digitalPinToInterrupt(RED_BUTTON), onRedButtonChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GREEN_BUTTON), onGreenButtonChange, CHANGE);

}

void loop() {

  switch (modeActuel) {
    case 1: modeStandard(); break;
    case 2: modeConfiguration(); break;
    case 3: modeMaintenance(); break;
    case 4: modeEconomique(); break;
  }
}

void onRedButtonChange() {
  if (digitalRead(RED_BUTTON) == LOW) {
    redPressed = true;
    redPressTime = millis();
  } else {
    if (redPressed && millis() - redPressTime >= 5000) {
      if (modeActuel == 1 || modeActuel == 4) {
        modePrecedent = modeActuel;
        modeActuel = 3;
        setLEDForMode(3);
      } else if (modeActuel == 3) {
        modeActuel = modePrecedent;
        setLEDForMode(modeActuel);
      }
    }
    redPressed = false;
  }
}

void onGreenButtonChange() {
  if (digitalRead(GREEN_BUTTON) == LOW) {
    greenPressed = true;
    greenPressTime = millis();
  } else {
    if (greenPressed && millis() - greenPressTime >= 5000 && modeActuel == 1) {
      modePrecedent = modeActuel;
      modeActuel = 4;
      setLEDForMode(4);
    }
    else if (greenPressed && millis() - greenPressTime >= 5000 && modeActuel == 4) {
      modePrecedent = modeActuel;
      modeActuel = 1;
      setLEDForMode(1);
    }
    greenPressed = false;
  }
}

void loadConfigFromEEPROM() {
  if (EEPROM.read(EEPROM_LOG_INTERVAL_ADDR) == 0xFF) {
    writeDefaultsToEEPROM();
  }
  LOG_INTERVAL = EEPROM.read(EEPROM_LOG_INTERVAL_ADDR);
  EEPROM.get(EEPROM_FILE_MAX_SIZE_ADDR, FILE_MAX_SIZE);
  TIMEOUT = EEPROM.read(EEPROM_TIMEOUT_ADDR);
  LUMIN = EEPROM.read(EEPROM_LUMIN_ADDR);
  EEPROM.get(EEPROM_LUMIN_LOW_ADDR, LUMIN_LOW);
  EEPROM.get(EEPROM_LUMIN_HIGH_ADDR, LUMIN_HIGH);
  TEMP_AIR = EEPROM.read(EEPROM_TEMP_AIR_ADDR);
  EEPROM.get(EEPROM_MIN_TEMP_AIR_ADDR, MIN_TEMP_AIR);
  EEPROM.get(EEPROM_MAX_TEMP_AIR_ADDR, MAX_TEMP_AIR);
  HYGR = EEPROM.read(EEPROM_HYGR_ADDR);
  EEPROM.get(EEPROM_HYGR_MINT_ADDR, HYGR_MINT);
  EEPROM.get(EEPROM_HYGR_MAXT_ADDR, HYGR_MAXT);
  PRESSURE = EEPROM.read(EEPROM_PRESSURE_ADDR);
  EEPROM.get(EEPROM_PRESSURE_MIN_ADDR, PRESSURE_MIN);
  EEPROM.get(EEPROM_PRESSURE_MAX_ADDR, PRESSURE_MAX);
}

void writeDefaultsToEEPROM() {
  EEPROM.update(EEPROM_LOG_INTERVAL_ADDR, DEF_LOG_INTERVAL);
  EEPROM.put(EEPROM_FILE_MAX_SIZE_ADDR, DEF_FILE_MAX_SIZE);
  EEPROM.update(EEPROM_TIMEOUT_ADDR, DEF_TIMEOUT);
  EEPROM.update(EEPROM_LUMIN_ADDR, DEF_LUMIN);
  EEPROM.put(EEPROM_LUMIN_LOW_ADDR, DEF_LUMIN_LOW);
  EEPROM.put(EEPROM_LUMIN_HIGH_ADDR, DEF_LUMIN_HIGH);
  EEPROM.update(EEPROM_TEMP_AIR_ADDR, DEF_TEMP_AIR);
  EEPROM.put(EEPROM_MIN_TEMP_AIR_ADDR, DEF_MIN_TEMP_AIR);
  EEPROM.put(EEPROM_MAX_TEMP_AIR_ADDR, DEF_MAX_TEMP_AIR);
  EEPROM.update(EEPROM_HYGR_ADDR, DEF_HYGR);
  EEPROM.put(EEPROM_HYGR_MINT_ADDR, DEF_HYGR_MINT);
  EEPROM.put(EEPROM_HYGR_MAXT_ADDR, DEF_HYGR_MAXT);
  EEPROM.update(EEPROM_PRESSURE_ADDR, DEF_PRESSURE);
  EEPROM.put(EEPROM_PRESSURE_MIN_ADDR, DEF_PRESSURE_MIN);
  EEPROM.put(EEPROM_PRESSURE_MAX_ADDR, DEF_PRESSURE_MAX);
}

void setLEDForMode(uint8_t mode) {
  switch (mode) {
    case 1: leds.setColorRGB(0, 0, 255, 0); break; // Couleur Verte
    case 2: leds.setColorRGB(0, 255, 255, 0); break; // Couleur Jaune
    case 3: leds.setColorRGB(0, 255, 165, 0); break; // Couleur orange
    case 4: leds.setColorRGB(0, 0, 0, 255); break; // Bleu
    default: leds.setColorRGB(0, 0, 255, 0); break;  //Verte, couleur par defaut
  }
}

void indicateError(uint8_t code) {
  while (true) {
    switch (code) {
      case 1: // RTC error
        leds.setColorRGB(0, 255, 0, 0); delay(500);
        leds.setColorRGB(0, 0, 0, 255); delay(500);
        break;
      case 2: // GPS error
        leds.setColorRGB(0, 255, 0, 0); delay(500);
        leds.setColorRGB(0, 255, 60, 0); delay(500);
        break;
      case 3: // Sensor error
        leds.setColorRGB(0, 255, 0, 0); delay(500);
        leds.setColorRGB(0, 0, 255, 0); delay(500);
        break;
      case 4: // Sensor incoherent
        leds.setColorRGB(0, 255, 0, 0); delay(500);
        leds.setColorRGB(0, 0, 255, 0); delay(1000);
        break;
      case 5: // SD full
        leds.setColorRGB(0, 255, 0, 0); delay(500);
        leds.setColorRGB(0, 255, 255, 255); delay(500);
        break;
      case 6: // SD write error
        leds.setColorRGB(0, 255, 0, 0); delay(500);
        leds.setColorRGB(0, 255, 255, 255); delay(1000);
        break;
      default:
        leds.setColorRGB(0, 255, 0, 0); delay(500);
        leds.setColorRGB(0, 255, 255, 255); delay(500);
        break;
    }
  }
}

void modeStandard() {
Serial.println(F("--- Standard Mode ---"));
setLEDForMode(1);
  static unsigned long interval_ms = 0;
  if (interval_ms == 0) interval_ms = (unsigned long)LOG_INTERVAL * 60UL * 1000UL;
  if (millis() - lastMeasureTime >= interval_ms) {
    lastMeasureTime = millis();
    save_data();
  }
}

void modeConfiguration() {
    Serial.println(F("--- Configuration Mode ---"));
  setLEDForMode(2);
  if (Serial.available() > 0) {
    char buffer[32];
    int bytesRead = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[bytesRead] = '\0';
    processConfigurationCommand(buffer);
    lastConfigActivity = millis();
  }
  if (millis() - lastConfigActivity >= 30UL * 60UL * 1000UL) {
    modeActuel = 1;
    setLEDForMode(1);
  }
}

void modeMaintenance() {
    Serial.println(F("--- Maintenance Mode ---"));
  setLEDForMode(3);
    Serial.println(F("--- Maintenance ---"));
    Serial.print(F("Loc: ")); Serial.println(get_localisation());
    Serial.print(F("Temp: ")); Serial.println(get_temp());
    Serial.print(F("Hum: ")); Serial.println(get_humidity());
    Serial.print(F("Pres: ")); Serial.println(get_pressure());
    Serial.print(F("Lum: ")); Serial.println(get_luminosity());
    DateTime now = rtc.now();
    char t[20];
    snprintf(t, sizeof(t), "%02d/%02d/%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    Serial.println(t);
    Serial.println(F("SD card can be removed."));
  delay(2000);
}

void modeEconomique() {
    Serial.println(F("--- Economic Mode ---"));
  setLEDForMode(4);
  static unsigned long interval_ms = 0;
  if (interval_ms == 0) interval_ms = (unsigned long)LOG_INTERVAL * 60UL * 1000UL * 2UL;
  if (millis() - lastMeasureTime >= interval_ms) {
    lastMeasureTime = millis();
    save_data();
  }
}

float get_luminosity() {
  int sensorValue = analogRead(LIGHT_SENSOR);
  return sensorValue; 


float get_temp()
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum);
  return temp;
}

float get_humidity()
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum);
  return hum;
}

float get_pressure()
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum);
  return pres;
}

String get_localisation() {
  if (modeActuel == 4) {
    eco_gps_skip = !eco_gps_skip;
    if (eco_gps_skip) return F("NA (ECO)");
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (SoftSerial.available()) {
      String gpsData = SoftSerial.readStringUntil('\n');
      gpsData.trim();

      if (gpsData.startsWith("$GPGGA") || gpsData.startsWith("$GNGGA")) {
        int fieldIndex = 0;
        String latitude, longitude;
        char latDir = ' ', lonDir = ' ';
        int start = 0;

        for (int i = 0; i < (int)gpsData.length(); i++) {
          if (gpsData[i] == ',' || i == (int)gpsData.length() - 1) {
            String token = gpsData.substring(start, i);
            start = i + 1;
            fieldIndex++;

            if (fieldIndex == 3) latitude = token;
            else if (fieldIndex == 4 && token.length() > 0) latDir = token[0];
            else if (fieldIndex == 5) longitude = token;
            else if (fieldIndex == 6 && token.length() > 0) lonDir = token[0];
          }
        }

        if (latitude.length() && longitude.length()) {
          return latitude + latDir + "," + longitude + lonDir;
        }
      }
    }
    delay(10); // soulage le microcontrôleur
  }

  return F("");
}


void applyFilenameForNow(uint8_t revisionZero) {
  DateTime now = rtc.now();
  snprintf(baseFilename, sizeof(baseFilename), "%02d%02d%02d_%d.LOG", now.year() % 100, now.month(), now.day(), revisionZero);
}

void save_data() {
  applyFilenameForNow(0);
  currentFile = SD.open(baseFilename, FILE_WRITE);
  if (!currentFile) {
    Serial.println(F("Failed to open file"));
    indicateError(6);
    return;
  }
  DateTime now = rtc.now();
  char timeBuffer[20];
  snprintf(timeBuffer, sizeof(timeBuffer), "%02d%02d%02d %02d:%02d:%02d", now.year() % 100, now.month(), now.day(), now.hour(), now.minute(), now.second());
  currentFile.print(F("Loc: ")); currentFile.print(get_localisation());
  currentFile.print(F("; Temp: ")); currentFile.print(get_temp());
  currentFile.print(F("; Hum: ")); currentFile.print(get_humidity());
  currentFile.print(F("; Pres: ")); currentFile.print(get_pressure());
  currentFile.print(F("; Lum: ")); currentFile.print(get_luminosity());
  currentFile.print(F("; ")); currentFile.println(timeBuffer);
  currentFile.flush();
  if (currentFile.size() == 0) indicateError(5);
  currentFile.close();
}

void processConfigurationCommand(char* input) {
  char* eq = strchr(input, '=');
  if (eq) {
    *eq = '\0';
    char* cmd = input;
    char* valueStr = eq + 1;
    int value = atoi(valueStr);
    if (strcmp(cmd, "LOG_INTERVAL") == 0) {
      LOG_INTERVAL = constrain(value, 1, 255);
      EEPROM.update(EEPROM_LOG_INTERVAL_ADDR, LOG_INTERVAL);
    } else if (strcmp(cmd, "FILE_MAX_SIZE") == 0) {
      FILE_MAX_SIZE = constrain(value, 0, 65535);
      EEPROM.put(EEPROM_FILE_MAX_SIZE_ADDR, FILE_MAX_SIZE);
    } else if (strcmp(cmd, "TIMEOUT") == 0) {
      TIMEOUT = constrain(value, 0, 255);
      EEPROM.update(EEPROM_TIMEOUT_ADDR, TIMEOUT);
    } else if (strcmp(cmd, "LUMIN") == 0) {
      LUMIN = constrain(value, 0, 1);
      EEPROM.update(EEPROM_LUMIN_ADDR, LUMIN);
    } else if (strcmp(cmd, "LUMIN_LOW") == 0) {
      LUMIN_LOW = constrain(value, 0, 1023);
      EEPROM.put(EEPROM_LUMIN_LOW_ADDR, LUMIN_LOW);
    } else if (strcmp(cmd, "LUMIN_HIGH") == 0) {
      LUMIN_HIGH = constrain(value, 0, 1023);
      EEPROM.put(EEPROM_LUMIN_HIGH_ADDR, LUMIN_HIGH);
    } else if (strcmp(cmd, "RESET") == 0) {
      writeDefaultsToEEPROM();
      loadConfigFromEEPROM();
    }
    // ... autres commandes
  }
}
