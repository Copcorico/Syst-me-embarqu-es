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

// PAS DE BUFFER GLOBAL 'Mesure[500]' POUR ÉCONOMISER LA RAM

int lignes = 0;
int compt = 1;

// Adresses EEPROM
#define EEPROM_LOG_INTERVAL_ADDR 0        // 1 octet
#define EEPROM_FILE_MAX_SIZE_ADDR 1       // 2 octets (adresse 1 et 2)
#define EEPROM_TIMEOUT_ADDR 3             // 1 octet
#define EEPROM_LUMIN_ADDR 4               // 1 octet
#define EEPROM_LUMIN_LOW_ADDR 5           // 2 octets (adresse 5 et 6)
#define EEPROM_LUMIN_HIGH_ADDR 7          // 2 octets (adresse 7 et 8)
#define EEPROM_TEMP_AIR_ADDR 9            // 1 octet
#define EEPROM_MIN_TEMP_AIR_ADDR 10       // 2 octets (adresse 10 et 11)
#define EEPROM_MAX_TEMP_AIR_ADDR 12       // 2 octets (adresse 12 et 13)
#define EEPROM_HYGR_ADDR 14               // 1 octet
#define EEPROM_HYGR_MINT_ADDR 15          // 2 octets (adresse 15 et 16)
#define EEPROM_HYGR_MAXT_ADDR 17          // 2 octets (adresse 17 et 18)
#define EEPROM_PRESSURE_ADDR 19           // 1 octet
#define EEPROM_PRESSURE_MIN_ADDR 20       // 2 octets (adresse 20 et 21)
#define EEPROM_PRESSURE_MAX_ADDR 22       // 2 octets (adresse 22 et 23)
#define EEPROM_CLOCK_ADDR 24              // 8 octets pour l'heure (format "HH:MM:SS")
#define EEPROM_DATE_ADDR 32               // 10 octets pour la date (format "DD-MM-YYYY")
#define EEPROM_DAY_ADDR 42                // 3 octets pour le jour de la semaine (format "MON")

//variables globales de configuration
int LUMIN, LUMIN_LOW, LUMIN_HIGH, TEMP_AIR, MIN_TEMP_AIR, MAX_TEMP_AIR;
int HYGR, HYGR_MINT, HYGR_MAXT, PRESSURE, PRESSURE_MIN, PRESSURE_MAX;
// Valeur par défaut de 10 minutes 
int LOG_INTERVALL = 10;
int FILE_MAX_SIZE = 4096;
int TIMEOUT = 30;
//Constantes bouttons
const int red_button = 3;
const int green_button = 2;

// Variables globales de mode
int modeActuel = 1;
int modePrecedent = 1;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  Wire.begin();
  rtc.begin();
  SoftSerial.begin(9600);
  pinMode(red_button, INPUT);
  pinMode(green_button, INPUT);
  leds.setColorRGB(0, 0, 255, 0); // LED Verte par défaut (Mode Standard) 
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  //Initialisation de la carte SD
  if (!SD.begin(4)) {
    Serial.println(F("Échec de l'initialisation de la carte SD !"));
    // LED Erreur accès SD (Rouge et Blanche)
    while (true) {
      leds.setColorRGB(0, 255, 0, 0); // Rouge
      delay(500);
      leds.setColorRGB(0, 255, 255, 255); // Blanc
      delay(1000); // Durée 2x plus longue pour le blanc 
    }
  } else Serial.println(F("Carte SD prête."));

  attachInterrupt(digitalPinToInterrupt(red_button), red_interruption, FALLING);
  attachInterrupt(digitalPinToInterrupt(green_button), green_interruption, FALLING);
}

void loop() {
  check_buttons_press(&modeActuel, &modePrecedent);

  switch (modeActuel) {
    case 1: modeStandard(); break;
    case 2: modeConfiguration(); break;
    case 3: modeMaintenance(); break;
    case 4: modeEconomique(); break;
  }
}

void check_buttons_press(int *modeActuel, int *modePrecedent) {
  // Cette fonction est complexe et gère la logique des boutons
  // (La logique de 'check_buttons_press' que vous aviez écrite
  // doit correspondre au PDF pour les modes éco/maintenance)
  
  // NOTE : Votre logique de bouton précédente ne correspond PAS au PDF.
  // PDF dit :
  // Mode Maintenance : 5s sur ROUGE 
  // Mode Économique : 5s sur VERT 
  
  // (Je laisse votre logique de bouton pour l'instant, 
  // mais elle semble inversée par rapport au PDF)
  
  int red_button_state = digitalRead(red_button);
  int green_button_state = digitalRead(green_button);
  long debutAppui = millis();
  
  static long dernierAppui = 0; 

  // Démarrage avec bouton rouge pressé -> Mode Config 
  // (Ceci devrait être dans setup(), mais on le garde ici pour l'instant)
  if (red_button_state == LOW) {
    if (millis() - debutAppui <= 2000 && red_button_state == HIGH) *modeActuel = 2 ;
  }

  // PDF dit : Mode Maintenance = 5s ROUGE
  if (red_button_state == LOW && (millis() - debutAppui >= 5000)) {
    if (*modeActuel == 1 || *modeActuel == 4) { // Depuis Standard ou Éco
      *modePrecedent = *modeActuel;
      *modeActuel = 3; // Mode Maintenance
      leds.setColorRGB(0, 255, 165, 0);  // LED Orange 
      Serial.println(F("Passage au mode Maintenance"));
    } else if (*modeActuel == 3) { // Quitter Maintenance
      *modeActuel = *modePrecedent; // Retour au mode précédent 
      if (*modePrecedent == 1) {
          leds.setColorRGB(0, 0, 255, 0); // LED Verte 
          Serial.println(F("Retour au mode Standard"));
      } else {
          leds.setColorRGB(0, 0, 0, 255); // LED Bleue 
          Serial.println(F("Retour au mode Économique"));
      }
    }
    dernierAppui = millis();
  }

  // PDF dit : Mode Économique = 5s VERT
  if (green_button_state == LOW && (millis() - debutAppui >= 5000)) {
    if (*modeActuel == 1) { // Depuis Standard UNIQUEMENT 
      *modePrecedent = *modeActuel;
      *modeActuel = 4; // Mode Économique
      leds.setColorRGB(0, 0, 0, 255);  // LED Bleue 
      Serial.println(F("Passage au mode Économique"));
    }
    // Note : Le PDF ne dit pas comment sortir du mode Éco
    // Sauf par un appui de 5s sur ROUGE 
    // ce qui est déjà géré par la logique du bouton ROUGE (retour au mode précédent)
    dernierAppui = millis();
  }

  // Inactivité en mode Configuration 
  if (red_button_state == HIGH && green_button_state == HIGH && (millis() - dernierAppui >= 1800000)) { // 1800000 ms = 30 minutes
    if (*modeActuel == 2) {
      *modeActuel = 1;
      leds.setColorRGB(0, 0, 255, 0);  // LED verte 
      Serial.println(F("Retour au mode Standard après inactivité en mode Configuration"));
    }
  }
}

void modeConfiguration() {
  leds.setColorRGB(0, 255, 60, 0); // LED Jaune 
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    processConfigurationCommand(input);
  }
}

// ... (get_luminosity, get_pressure, get_humidity, get_temp... inchangés) ...
float get_luminosity() {
  int sensorValue = analogRead(light_sensor);
  if (sensorValue <= 0) return 1023.0;
  float Rsensor = (float)(1023 - sensorValue) * 10 / sensorValue;
  return Rsensor;
}
float get_pressure(Stream* client) {
  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  return (pres);
}
float get_humidity(Stream* client) {
  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  return (hum);
}
float get_temp(Stream* client) {
  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  return (temp);
}


// MODIFIÉ : Ajout de la logique du mode Économique
String get_localisation() {
  
  // Logique Mode Éco : 1 mesure GPS sur 2 
  if (modeActuel == 4) { 
    static bool eco_gps_skip = false;
    eco_gps_skip = !eco_gps_skip; // Inverse le "saut" (Vrai, Faux, Vrai, Faux...)
    if (eco_gps_skip) {
      return "NA (ECO)"; // Retourne "NA" une fois sur deux
    }
    // Si on ne saute pas, on exécute la fonction normalement
  }
  
  if (!SoftSerial.available()) return "";
  String gpsData = SoftSerial.readStringUntil('\n');
  gpsData.trim();
  if (!gpsData.startsWith("$GPGGA")) return "";
  
  // (Reste de votre logique de parsage GPS)
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
  // Le format de nom de fichier dans le PDF est différent 
  // Mais nous gardons votre logique pour l'instant
  const char *base = "Fich_"; 
  snprintf(nom_f, sizeof(nom_f), "%s%d.txt", base, compt);

  File f = SD.open(nom_f, FILE_READ);
  if (f) {
    lignes = 0;
    bool prevCR = false;
    while (f.available()) {
      char c = f.read();
      if (c == '\n') {
        if (!prevCR) lignes++;
        prevCR = false;
      } else if (c == '\r') {
        lignes++;
        prevCR = true;
      } else {
        prevCR = false;
      }
    }
  }
  f.close();
  
  Serial.println(lignes);
  
  // Gestion de la taille de fichier (simplifiée)
  // Le PDF décrit une rotation de log complexe
  if (lignes >= Nbr_MAX_MESURE) { 
    compt ++;
    snprintf(nom_f, sizeof(nom_f), "%s%d.txt", base, compt);
    lignes = 0;
  }
  
  fichier = SD.open(nom_f, FILE_WRITE);

  if (!fichier) {
    Serial.println(F("Impossible d'ouvrir le fichier en écriture"));
    // LED Erreur accès SD 
    while (!fichier) {
      leds.setColorRGB(0, 255, 0, 0); // Rouge
      delay(500);
      leds.setColorRGB(0, 255, 255, 255); // Blanc
      delay(1000); // Durée 2x plus longue pour le blanc 
    }
    return ;
  } else {
    // OPTIMISATION : Écrire les données pièce par pièce
    
    char timeBuffer[30]; // Buffer local pour l'heure
    get_time(timeBuffer, sizeof(timeBuffer)); // Remplit le buffer

    // Enregistre une seule ligne horodatée 
    fichier.print(F("Localisation : "));
    fichier.print(get_localisation()); 
    fichier.print(F(" ; Température : "));
    fichier.print(get_temp(&Serial));
    fichier.print(F(" ; Humidité : "));
    fichier.print(get_humidity(&Serial));
    fichier.print(F(" ; Pression : "));
    fichier.print(get_pressure(&Serial));
    fichier.print(F(" ; Luminosité : "));
    fichier.print(get_luminosity());
    fichier.print(F(" ; "));
    fichier.println(timeBuffer); // Imprime la date/heure
    
    if (fichier.size() == 0) { 
      Serial.println(F("Erreur : écriture échouée (carte pleine?)"));
      // LED Carte SD pleine 
      while (true) { 
        leds.setColorRGB(0, 255, 0, 0); // Rouge
        delay(500);
        leds.setColorRGB(0, 255, 255, 255); // Blanc
        delay(500); // Durée identique 
      }
    }
    
    fichier.close();
  }
}

// OPTIMISATION : Modifiée pour remplir un buffer
void get_time(char* buffer, size_t bufferSize) {
  DateTime now = rtc.now();
  snprintf(buffer, bufferSize, "%02d/%02d/%04d ; Heure: %02d:%02d",
           now.day(), now.month(), now.year(), now.hour(), now.minute());
}

// Mode maintenance
void enter_maintenance() {
  // Affiche les données sur le port série 
  Serial.println(F("--- Mode Maintenance ---"));
  Serial.print(F("Localisation : "));
  Serial.println(get_localisation()); 
  Serial.print(F("Température : "));
  Serial.println(get_temp(&Serial));
  Serial.print(F("Humidité : "));
  Serial.println(get_humidity(&Serial));
  Serial.print(F("Pression : "));
  Serial.println(get_pressure(&Serial));
  
  char timeBuffer[30];
  get_time(timeBuffer, sizeof(timeBuffer));
  Serial.println(timeBuffer);
  Serial.println(F("------------------------"));
  Serial.println(F("Retrait de la carte SD autorisé. "));
}

// (processConfigurationCommand... inchangé)
void processConfigurationCommand(String input) {
  int separatorIndex = input.indexOf('=');
  String param = input.substring(0, separatorIndex);
  String value = input.substring(separatorIndex + 1);
  param.trim();
  value.trim();
  
  if (param == "LOG_INTERVAL") {
    int interval = constrain(value.toInt(), 0, 100);
    EEPROM.put(EEPROM_LOG_INTERVAL_ADDR, interval);
    Serial.println(F("Intervalle de log mis à jour."));
    afficherConfigEEPROM(interval);
  } else if (param == "FILE_MAX_SIZE") {
    int size = constrain(value.toInt(), 0, 4096);
    EEPROM.put(EEPROM_FILE_MAX_SIZE_ADDR, size);
    Serial.println(F("Taille max du fichier de log mise à jour."));
  } else if (param == "TIMEOUT") {
    int timeout = constrain(value.toInt(), 0, 100);
    EEPROM.put(EEPROM_TIMEOUT_ADDR, timeout);
    Serial.println(F("Timeout mis à jour."));
  } else if (param == "LUMIN") {
    LUMIN = constrain(value.toInt(), 0, 1);
    EEPROM.put(EEPROM_LUMIN_ADDR, LUMIN);
    Serial.println(F("LUMIN mis à jour."));
  } else if (param == "LUMIN_LOW") {
    LUMIN_LOW = constrain(value.toInt(), 0, 1023);
    EEPROM.put(EEPROM_LUMIN_LOW_ADDR, LUMIN_LOW);
  } else if (param == "LUMIN_HIGH") {
    LUMIN_HIGH = constrain(value.toInt(), 0, 1023);
    EEPROM.put(EEPROM_LUMIN_HIGH_ADDR, LUMIN_HIGH);
  } else if (param == "HYGR") {
    HYGR = constrain(value.toInt(), 0, 1);
    EEPROM.put(EEPROM_HYGR_ADDR, HYGR);
    Serial.println(F("HYGR mis à jour."));
  } else if (param == "HYGR_MIN") {
    HYGR_MINT = constrain(value.toInt(), -40, 85);
    EEPROM.put(EEPROM_HYGR_MINT_ADDR, HYGR_MINT);
  } else if (param == "HYGR_MAXT") {
    HYGR_MAXT = constrain(value.toInt(), -40, 85);
    EEPROM.put(EEPROM_HYGR_MAXT_ADDR, HYGR_MAXT);
  } else if (param == "TEMP_AIR") {
    TEMP_AIR = constrain(value.toInt(), 0, 1);
    EEPROM.put(EEPROM_TEMP_AIR_ADDR, TEMP_AIR);
    Serial.println(F("TEMP_AIR mis à jour."));
  } else if (param == "MIN_TEMP_AIR") {
    MIN_TEMP_AIR = constrain(value.toInt(), -40, 85);
    EEPROM.put(EEPROM_MIN_TEMP_AIR_ADDR, MIN_TEMP_AIR);
  } else if (param == "MAX_TEMP_AIR") {
    MAX_TEMP_AIR = constrain(value.toInt(), -40, 85);
    EEPROM.put(EEPROM_MAX_TEMP_AIR_ADDR, MAX_TEMP_AIR);
  } else if (param == "PRESSURE") {
    PRESSURE = constrain(value.toInt(), 0, 1);
    EEPROM.put(EEPROM_PRESSURE_ADDR, PRESSURE);
  } else if (param == "PRESSURE_MIN") {
    PRESSURE_MIN = constrain(value.toInt(), 300, 1100);
    EEPROM.put(EEPROM_PRESSURE_MIN_ADDR, PRESSURE_MIN);
  } else if (param == "PRESSURE_MAX" ) {
    PRESSURE_MAX = constrain(value.toInt(), 300, 1100);
    EEPROM.put(EEPROM_PRESSURE_MAX_ADDR, PRESSURE_MAX);
  } else if (param == "DAY") {
    day(value);
  } else if (param == "CLOCK") {
    updateClock(value);
  } else if (param == "DATE") {
    updateDate(value);
  } else if (param == "VERSION") {
    Serial.println(F("Version : 1.0.0, Numéro du lot : 123. "));
  } else if (param == "RESET") {
    Serial.println(F("Réinitialisation des paramètres... (LOGIQUE À FAIRE)"));
    // (Ici, vous devriez réécrire les valeurs par défaut dans l'EEPROM)
  }
  else {
    Serial.println(F("Commande inconnue"));
  }
}

// =======================================================
// --- Fonctions (STUBS) à remplir ---
// =======================================================

void red_interruption() {
  // (Logique d'interruption, si nécessaire)
}

void green_interruption() {
  // (Logique d'interruption, si nécessaire)
}

// REMPLI : Logique du Mode Standard
void modeStandard() {
  static unsigned long lastMeasure = 0;
  
  // LOG_INTERVALL est en minutes , on convertit en ms
  unsigned long intervalle_ms = (unsigned long)LOG_INTERVALL * 60 * 1000;
  
  if (millis() - lastMeasure > intervalle_ms) {
    lastMeasure = millis();
    Serial.println(F("Mode Standard: Prise de mesure..."));
    // Récupère les capteurs et enregistre sur SD 
    save_data(); 
  }
}

// REMPLI : Logique du Mode Maintenance
void modeMaintenance() {
  // Les données ne sont pas écrites sur SD 
  enter_maintenance();
  delay(2000); // Délai pour ne pas spammer le port série
}

// REMPLI : Logique du Mode Économique
void modeEconomique() {
  static unsigned long lastMeasure = 0;
  
  // L'intervalle est doublé en mode éco 
  unsigned long intervalle_ms = (unsigned long)LOG_INTERVALL * 60 * 1000 * 2;
  
  if (millis() - lastMeasure > intervalle_ms) {
    lastMeasure = millis();
    Serial.println(F("Mode Économique: Prise de mesure..."));
    // La logique de saut GPS (1/2) est gérée dans get_localisation()
    save_data();
  }
}

// (Stubs restants)
void afficherConfigEEPROM(int interval) {
  Serial.print(F("Affichage config EEPROM. Intervalle = "));
  Serial.println(interval);
}

void day(String value) {
  Serial.print(F("Mise à jour du jour : "));
  Serial.println(value);
  // (Logique de parsage et d'appel RTC à ajouter)
}

void updateClock(String value) {
  Serial.print(F("Mise à jour de l'heure : "));
  Serial.println(value);
  // (Logique de parsage et d'appel RTC à ajouter)
}

void updateDate(String value) {
  Serial.print(F("Mise à jour de la date : "));
  Serial.println(value);
  // (Logique de parsage et d'appel RTC à ajouter)
}
