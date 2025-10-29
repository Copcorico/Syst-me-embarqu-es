#include <ChainableLED.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>

  
// Initialisation des composants
BME280I2C bme;
RTC_DS3231 rtc;
ChainableLED leds(5, 6, 1);
SoftwareSerial SoftSerial(8, 9);
#define SERIAL_BAUD 9600
#define light_sensor A0
// Carte SD et sauvegarde
File fichier;
const int Nbr_MAX_MESURE = 8;
char *Mesure[500];
int lignes = 0;
int compt = 1;

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
//variables globales de configuration
int LUMIN, LUMIN_LOW, LUMIN_HIGH, TEMP_AIR, MIN_TEMP_AIR, MAX_TEMP_AIR;
int HYGR, HYGR_MINT, HYGR_MAXT, PRESSURE, PRESSURE_MIN, PRESSURE_MAX;
int LOG_INTERVALL = 10, FILE_MAX_SIZE = 4096, TIMEOUT = 30;
//Constantes bouttons
const int red_button = 3;
const int green_button = 2;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  Wire.begin();
  rtc.begin();
  SoftSerial.begin(9600);
  int modeActuel=1;
  int modePrecedent=1; // Variable pour stocker le mode précédent
  pinMode(red_button, INPUT);
  pinMode(green_button, INPUT);
  leds.setColorRGB(0, 0, 255, 0);
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //Initialisation de la carte SD
  if (!SD.begin(4)) {  // 4 = broche CS (selon le module)
    Serial.println("Échec de l'initialisation de la carte SD !");
    exit(1);
    return;
  } else Serial.println("Carte SD prête.");

  attachInterrupt(digitalPinToInterrupt(red_button), red_interruption, FALLING);
  attachInterrupt(digitalPinToInterrupt(green_button), green_interruption, FALLING);
}

void loop() {
  check_buttons_press(&modeActuel,&modePrecedent);


  switch (modeActuel) {
  case 1: modeStandard(); break;
  case 2: modeConfiguration(); break;
  case 3: modeMaintenance(); break;
  case 4: modeEconomique(); break;
  }
}
void check_buttons_press(int *modeActuel, int *modePrecedent) {
int red_button_state = digitalRead(red_button);
int green_button_state = digitalRead(green_button);
long debutAppui = millis();
long dernierAppui ;
  
  //Bouton rouge cliqué, mode configuration
  if (red_button_state == LOW) {
    if (millis() - debutAppui <=2000 && red_button_state == HIGH) *modeActuel = 2 ;
  }
  // Bouton Rouge (retour direct au mode Standard depuis Maintenance ou Économique)
  if (red_button_state == LOW && (millis() - debutAppui >= 5000)) {
    if (*modeActuel == 3 || *modeActuel == 4) {
      // Retour direct au mode Standard depuis le mode Maintenance ou Économique
      *modeActuel = 1;
      leds.setColorRGB(0, 0, 255, 0);  // LED verte continue pour le mode Standard
      Serial.println("Retour direct au mode Standard depuis Maintenance ou Économique");
    } else if (*modeActuel == 1) {
      // Passer du mode Standard au mode Économique
      *modePrecedent = *modeActuel;
      *modeActuel = 4;
      leds.setColorRGB(0, 0, 0, 255);  // LED bleue continue pour le mode Économique
      Serial.println("Passage au mode Économique");
    }
    dernierAppui = millis();  // Mettre à jour le dernier appui
  }

  // Bouton Vert (Mode Maintenance <-> Retour au mode précédent ou mode Standard)
  if (green_button_state == LOW && (millis() - debutAppui >= 5000)) {
    if (*modeActuel == 1 || *modeActuel == 4) {
      // Passer en mode Maintenance depuis le mode Standard ou Économique
      *modePrecedent = *modeActuel;
      *modeActuel = 3;
      leds.setColorRGB(0, 255, 165, 0);  // LED orange continue pour le mode Maintenance
      Serial.println("Passage au mode Maintenance");
    } else if (*modeActuel == 3) {
      // Retourner au mode précédent (Standard ou Économique) depuis le mode Maintenance
      *modeActuel = *modePrecedent;
      if (*modePrecedent == 1) {
        leds.setColorRGB(0, 0, 255, 0);  // LED verte continue pour le mode Standard
        Serial.println("Retour au mode Standard depuis Maintenance");
      } else if (*modePrecedent == 4) {
        leds.setColorRGB(0, 0, 0, 255);  // LED bleue continue pour le mode Économique
        Serial.println("Retour au mode Économique depuis Maintenance");
      }
    }
    dernierAppui = millis();  // Mettre à jour le dernier appui
  }

  // Vérification d'inactivité en mode Configuration
  if (red_button_state == HIGH && green_button_state == HIGH && (millis() - dernierAppui >= 1800000)) {
    if (*modeActuel == 2) {
      // Retour automatique au mode Standard après inactivité
      *modeActuel = 1;
      leds.setColorRGB(0, 0, 255, 0);  // LED verte continue pour le mode standard
      Serial.println("Retour au mode Standard après inactivité en mode Configuration");
    }
  }

}
void modeConfiguration() {
  leds.setColorRGB(0, 255, 60, 0);
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    processConfigurationCommand(input);
  }
}


float get_luminosity() {
  int sensorValue = analogRead(light_sensor); // Lire la valeur brute du capteur
  if (sensorValue <= 0) return 1023.0; // Éviter la division par zéro
  float Rsensor=(float)(1023-sensorValue)*10/sensorValue;
  return Rsensor; // Retourner la valeur brute pour un traitement ultérieur

}

float get_pressure(Stream* client){
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   //client->print("\t\tPressure: ");
   //client->print(pres);
   //client->println(" Pa");
  return(pres);
}

float get_humidity
(
  Stream* client
)
  {
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

  //client->print("\t\tHumidity: ");
  //client->print(hum);
  //client->print("% RH");
  //client->print("\n");
  return(hum);
  }

float get_temp(Stream* client){
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   //client->print("\t\tPressure: ");
   //client->print(pres);
   //client->println(" Pa");
  return(temp);
}


String get_localisation() {
  if (!SoftSerial.available()) return "";
  String gpsData = SoftSerial.readStringUntil('\n');
  gpsData.trim();
  if (!gpsData.startsWith("$GPGGA")) return "";
  // découpage basique
  int fieldIndex = 0;
  String latitude = "";
  String longitude = "";
  char latDir = ' ';
  char lonDir = ' ';
  int start = 0;
  for (int i = 0; i < (int)gpsData.length(); i++) {
    if (gpsData[i] == ',' || i == (int)gpsData.length() - 1) {
      String token = gpsData.substring(start, i == (int)gpsData.length() - 1 ? i + 1 : i);
      start = i + 1;
      fieldIndex++;
      if (fieldIndex == 2) { /* UTC time, ignore */ }
      else if (fieldIndex == 3) latitude = token;
      else if (fieldIndex == 4 && token.length() > 0) latDir = token[0];
      else if (fieldIndex == 5) longitude = token;
      else if (fieldIndex == 6 && token.length() > 0) lonDir = token[0];
    }
  }
  
  if (latitude.length() && longitude.length()) {
    return latitude + latDir + "," + longitude + lonDir;
  }
  return "";
}


// Fonction de sauvegarde des données
void save_data() {
  char nom_f[20];
  const char *base = "Fich_";
  snprintf(nom_f, sizeof(nom_f),"%s%d.txt",base, compt);

  // Comptons le nombre de ligne de mesure presente
  File f = SD.open(nom_f, FILE_READ);
  if (f) {
    lignes = 0;
    bool prevCR = false;
    while (f.available()) {
      char c = f.read();
      if (c == '\n') {
        // Si on est en CRLF, on compte quand même une seule ligne
        if (!prevCR) lignes++;
        // reset
        prevCR = false;
      } else if (c == '\r') {
        // CR rencontré, on peut compter une ligne et attendre LF
        lignes++;
        prevCR = true;
      } else {
       prevCR = false;
      }
    }
  }
  f.close();
  Serial.println(lignes);
  if (lignes >= Nbr_MAX_MESURE){
    compt ++;
    snprintf(nom_f, sizeof(nom_f),"%s%d.txt",base, compt);
    lignes = 0;
  }
  // Création / ouverture d’un fichier
  fichier = SD.open(nom_f, FILE_WRITE);

  if (!fichier) {
    Serial.println("Impossible d'ouvrir le fichier en écriture");
    //LED intermittente rouge et blanche (fréquence 1Hz, durée 2 fois plus longue pourle blanc)
    while (!fichier) {
      leds.setColorRGB(0, 255, 0, 0); 
      delay(500);
      leds.setColorRGB(0, 255, 255, 255);
      delay(1000);
    }
    return ;
  } else {
  // ecriture dans le fichier
  snprintf(*Mesure, sizeof(Mesure), "Localisation : %s ; Température : %f ; Humidité : %f ; Pression : %f ; Luminosité : %f ; %s",get_localisation(), get_temp(&Serial), get_humidity(&Serial), get_pressure(&Serial), get_luminosity(), get_time());
  size_t written = fichier.println("%s",Mesure);
  fichier.flush(); // force l’écriture
  if (written == 0) {
    Serial.println("Erreur : carte SD probablement pleine !");
    
    //LED intermittente rouge et blanche (fréquence 1Hz, durée identique pour les 2 couleurs)
    while (written == 0){
      leds.setColorRGB(0, 255, 0, 0); 
      delay(500);
      leds.setColorRGB(0, 255, 255, 255);
      delay(500);
    }
  }
    
  fichier.close(); // Fermer le fihier
  }
}

String get_time() {
  DateTime now = rtc.now();
  char Date [80];
  snprintf(Date, sizeof(Date), "%02d/%02d/%04d ; Heure: %02d:%02d",
         now.day(), now.month(), now.year(), now.hour(), now.minute());
  return Date;
}

// Mode maintenance
void enter_maintenance(){
  Serial.println(Mesure);
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
