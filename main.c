#include <ChainableLED.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <RTClib.h>
#include <EEPROM.h>
// Initialisation des composants
BME280I2C bme;
RTC_DS3231 rtc;
ChainableLED leds(4, 5, 1);
SoftwareSerial gpsSerial(2, 3);
#define SERIAL_BAUD 9600

// Adresses EEPROM
// Adresses EEPROM réorganisées
#define EEPROM_LOG_INTERVAL_ADDR 0          // 1 octet
#define EEPROM_FILE_MAX_SIZE_ADDR 1         // 2 octets (adresse 1 et 2)
#define EEPROM_TIMEOUT_ADDR 3               // 1 octet

#define EEPROM_LUMIN_ADDR 4                 // 1 octet
#define EEPROM_LUMIN_LOW_ADDR 5             // 2 octets (adresse 5 et 6)
#define EEPROM_LUMIN_HIGH_ADDR 7            // 2 octets (adresse 7 et 8)

#define EEPROM_TEMP_AIR_ADDR 9              // 1 octet
#define EEPROM_MIN_TEMP_AIR_ADDR 10         // 2 octets (adresse 10 et 11)
#define EEPROM_MAX_TEMP_AIR_ADDR 12         // 2 octets (adresse 12 et 13)

#define EEPROM_HYGR_ADDR 14                 // 1 octet
#define EEPROM_HYGR_MINT_ADDR 15            // 2 octets (adresse 15 et 16)
#define EEPROM_HYGR_MAXT_ADDR 17            // 2 octets (adresse 17 et 18)

#define EEPROM_PRESSURE_ADDR 19             // 1 octet
#define EEPROM_PRESSURE_MIN_ADDR 20         // 2 octets (adresse 20 et 21)
#define EEPROM_PRESSURE_MAX_ADDR 22         // 2 octets (adresse 22 et 23)

#define EEPROM_CLOCK_ADDR 24                // 8 octets pour l'heure (format "HH:MM:SS")
#define EEPROM_DATE_ADDR 32                 // 10 octets pour la date (format "DD-MM-YYYY")
#define EEPROM_DAY_ADDR 42                  // 3 octets pour le jour de la semaine (format "MON")

const int red_button = 3;
const int green_button = 2;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  Wire.begin();
  rtc.begin();
    

  pinMode(red_button, INPUT);
  pinMode(green_button, INPUT);
  leds.setColorRGB(0, 0, 255, 0);

  attachInterrupt(digitalPinToInterrupt(bouton_bleu), interruption_bleu, FALLING);
  attachInterrupt(digitalPinToInterrupt(bouton_blanc), interruption_blanc, FALLING);
}

void loop() {
  int etatBoutonBleu = digitalRead(bouton_bleu);
  int etatBoutonBlanc = digitalRead(bouton_blanc);

  // Bouton Blanc (retour direct au mode Standard depuis Maintenance ou Économique)
  if (etatBoutonBleu == LOW && (millis() - dernierAppui >= 5000)) {
    if (modeActuel == MAINTENANCE || modeActuel == ECONOMIQUE) {
      // Retour direct au mode Standard depuis le mode Maintenance ou Économique
      modeActuel = STANDARD;
      leds.setColorRGB(0, 0, 255, 0);  // LED verte continue pour le mode Standard
      Serial.println("Retour direct au mode Standard depuis Maintenance ou Économique");
    } else if (modeActuel == STANDARD) {
      // Passer du mode Standard au mode Économique
      modePrecedent = modeActuel;
      modeActuel = ECONOMIQUE;
      leds.setColorRGB(0, 0, 0, 255);  // LED bleue continue pour le mode Économique
      Serial.println("Passage au mode Économique");
    }
    dernierAppui = millis();  // Mettre à jour le dernier appui
  }

  // Bouton Bleu (Mode Maintenance <-> Retour au mode précédent ou mode Standard)
  if (etatBoutonBlanc == LOW && (millis() - dernierAppui >= 5000)) {
    if (modeActuel == STANDARD || modeActuel == ECONOMIQUE) {
      // Passer en mode Maintenance depuis le mode Standard ou Économique
      modePrecedent = modeActuel;
      modeActuel = MAINTENANCE;
      leds.setColorRGB(255, 165, 0, 0);  // LED orange continue pour le mode Maintenance
      Serial.println("Passage au mode Maintenance");
    } else if (modeActuel == MAINTENANCE) {
      // Retourner au mode précédent (Standard ou Économique) depuis le mode Maintenance
      modeActuel = modePrecedent;
      if (modePrecedent == STANDARD) {
        leds.setColorRGB(0, 0, 255, 0);  // LED verte continue pour le mode Standard
        Serial.println("Retour au mode Standard depuis Maintenance");
      } else if (modePrecedent == ECONOMIQUE) {
        leds.setColorRGB(0, 0, 0, 255);  // LED bleue continue pour le mode Économique
        Serial.println("Retour au mode Économique depuis Maintenance");
      }
    }
    dernierAppui = millis();  // Mettre à jour le dernier appui
  }

  // Vérification d'inactivité en mode Configuration
  if (etatBoutonBleu == HIGH && etatBoutonBlanc == HIGH && (millis() - dernierAppui >= 1800000)) {
    if (modeActuel == CONFIGURATION) {
      // Retour automatique au mode Standard après inactivité
      modeActuel = STANDARD;
      leds.setColorRGB(0, 0, 255, 0);  // LED verte continue pour le mode standard
      Serial.println("Retour au mode Standard après inactivité en mode Configuration");
    }
  }
    switch (modeActuel) {
    case STANDARD: modeStandard(); break;
    case CONFIGURATION: modeConfiguration(); break;
    case MAINTENANCE: modeMaintenance(); break;
    case ECONOMIQUE: modeEconomique(); break;
  }
}


void modeConfiguration() {
  leds.setColorRGB(0, 255, 60, 0);
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    processConfigurationCommand(input);
  }
}

void processConfigurationCommand(String input) {
  int separatorIndex = input.indexOf('=');
  String param = input.substring(0, separatorIndex);
  String value = input.substring(separatorIndex + 1);
  param.trim();
  value.trim();
  if (param == "LOG_INTERVAL") {
    int interval = constrain(value.toInt(), 0, 100);  // Limite à la plage autorisée
    EEPROM.put(EEPROM_LOG_INTERVAL_ADDR, interval);
    Serial.println("Intervalle de log mis à jour.");
    afficherConfigEEPROM(interval); // exemple pour montrer que c bien stockés
  } else if (param == "FILE_MAX_SIZE") {
    int size = constrain(value.toInt(), 0, 4096);  // Limite la taille du fichier
    EEPROM.put(EEPROM_FILE_MAX_SIZE_ADDR, size);
    Serial.println("Taille max du fichier de log mise à jour.");
  }else if (param == "TIMEOUT") {
    int timeout = constrain(value.toInt(), 0, 100);  // Limite le timeout
    EEPROM.put(EEPROM_TIMEOUT_ADDR, timeout);
    Serial.println("Timeout mis à jour.");
  } else if (param == "LUMIN") {
    LUMIN = constrain(value.toInt(), 0, 1);
    EEPROM.put(EEPROM_LUMIN_ADDR, LUMIN);
    Serial.println("LUMIN mis à jour.");
  } else if (param == "LUMIN_LOW"){
    LUMIN_LOW = constrain(value.toInt(), 0, 1023);
    EEPROM.put(EEPROM_LUMIN_LOW_ADDR, LUMIN_LOW);
  } else if (param == "LUMIN_HIGH"){
    LUMIN_HIGH = constrain(value.toInt(), 0, 1023);
    EEPROM.put(EEPROM_LUMIN_HIGH_ADDR, LUMIN_HIGH);
  }else if (param == "HYGR"){
    HYGR = constrain(value.toInt(), 0, 1);
    EEPROM.put(EEPROM_HYGR_ADDR, HYGR);
    Serial.println("HYGR mis à jour.");
  }else if (param =="HYGR_MIN"){
    HYGR_MINT = constrain(value.toInt(), -40, 85);
    EEPROM.put(EEPROM_HYGR_MINT_ADDR, HYGR_MINT);
  }else if (param == "HYGR_MAXT"){
    HYGR_MAXT = constrain(value.toInt(), -40, 85);
    EEPROM.put(EEPROM_HYGR_MAXT_ADDR, HYGR_MAXT);
  }else if (param == "TEMP_AIR"){
    TEMP_AIR = constrain(value.toInt(), 0, 1);
    EEPROM.put(EEPROM_TEMP_AIR_ADDR, TEMP_AIR);
    Serial.println("TEMP_AIR mis à jour.");
  }else if (param =="MIN_TEMP_AIR"){
    MIN_TEMP_AIR = constrain(value.toInt(), -40, 85);
    EEPROM.put(EEPROM_MIN_TEMP_AIR_ADDR, MIN_TEMP_AIR);
  }else if (param == "MAX_TEMP_AIR"){
    MAX_TEMP_AIR = constrain(value.toInt(), -40, 85);
    EEPROM.put(EEPROM_MAX_TEMP_AIR_ADDR, MAX_TEMP_AIR);
  }else if (param == "PRESSURE"){
    PRESSURE = constrain(value.toInt(), 0, 1);
    EEPROM.put(EEPROM_PRESSURE_ADDR, PRESSURE);
   // Serial.println("PRESSURE mis à jour.");
  }else if (param == "PRESSURE_MIN"){
    PRESSURE_MIN = constrain(value.toInt(), 300, 1100);
    EEPROM.put(EEPROM_PRESSURE_MIN_ADDR, PRESSURE_MIN);
  }else if(param == "PRESSURE_MAX" ){
    PRESSURE_MAX = constrain(value.toInt(), 300, 1100);
    EEPROM.put(EEPROM_PRESSURE_MAX_ADDR, PRESSURE_MAX);
  }else if (param == "DAY"){
      day(value);
  }else if (param == "CLOCK"){
    updateClock(value);
  }else if (param == "DATE"){
    updateDate(value);
  }else if (param == "VERSION"){
    Serial.println("Version : 1.0.0, Numéro du lot : 123.");
  }else{
    Serial.println("Commande inconnue");
  }
}