#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <ESP32Encoder.h>

#define RayonRoue 0.0325                                                        // Rayon de la roue en m

// --- Configuration des broches moteurs ---
unsigned char PWMGplus = 17;                                                    // PWM moteur gauche (sens +)
unsigned char PWMDplus = 16;                                                    // PWM moteur droit  (sens +)
unsigned char PWMGmoins = 4;                                                    // PWM moteur gauche (sens -)
unsigned char PWMDmoins = 19;                                                   // PWM moteur droit  (sens -)
char Batterie = 25;                                                             // Mesure tension batterie

// --- Instanciation des objets ---
BluetoothSerial SerialBT;                                                       // Communication Bluetooth
Adafruit_MPU6050 mpu;                                                           // Capteur IMU
ESP32Encoder encoderL;                                                          // Encodeur Gauche
ESP32Encoder encoderR;                                                          // Encodeur Droit

// --- Variables de filtrage et temps réel ---
float TetaG, TetaW;                                                             // Angles bruts (Acc / Gyro)
float TetaWF, TetaGF, Teta;                                                     // Angles filtrés et fusionnés
char FlagCalcul = 0;                                                            // Flag de synchronisation
float Ve, Vs = 0;
float Te = 10.0;                                                                // Période d'échantillonnage (ms)
float Tau = 1000.0, TauVitesse = 570.0;                                         // Constantes de temps filtres
float A, B;                                                                     // Coeffs récurrence filtre
float AVitesse, BVitesse;

// --- Paramètres pont diviseur batterie ---
float R1 = 22000.0;                                                             // Résistance 22k
float R2 = 10000.0;                                                             // Résistance 10k
float valeurbatterie;

// --- Variables Odométrie ---
long TetaMG, TetaMD;
long encodeur_precedentMG = 0, encodeur_presentMG = 0;
long encodeur_precedentMD = 0, encodeur_presentMD = 0;
float deltaEncodeurMG, deltaEncodeurMD, deltaMoyenne;
float deltaEncodeurMG_Angulaire, deltaEncodeurMD_Angulaire;
float vitesseLineaire, vitesseLineaireF;
int TetaMG_par_Tick, TetaMD_par_Tick;
const float Nb_de_ticks = 748.0;                                                // Résolution par tour

// --- Paramètres des Correcteurs PID ---
float kpPosition = 3.19, kdPosition = 0.034;                                    // Gains boucle inclinaison
float erreurTeta;
volatile float TetaConsigne = 0.0;
float kpVitesse = 6.94, kdVitesse = 1.94;                                       // Gains boucle vitesse
float erreurVitesse, deriveVitesse = 0, erreurPrecedentVitesse = 0;
float VitesseConsigne = 0.0;
float Ec, CO1 = 0.166, CO2 = 0.06;                                              // Sortie et compensation frottement
int dutyCyclePositif, dutyCycleNegatif;
int offsetplusG, offsetplusD, offsetmoinsG, offsetmoinsD;                       // Offsets de trajectoire

// --- Configuration hardware PWM ---
unsigned int frequence = 20000;                                                 // 20 kHz
unsigned char MOTGplus = 0;                                                     // Canal PWM G+
unsigned char MOTDplus = 1;                                                     // Canal PWM D+
unsigned char MOTGmoins = 2;                                                    // Canal PWM G-
unsigned char MOTDmoins = 3;                                                    // Canal PWM D-
unsigned char resolution = 10;                                                  // Résolution 10 bits

// --- Tâche de contrôle temps réel ---
void controle(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Lecture capteur MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Filtrage inclinaison (Accéléromètre)
    TetaG = -atan2(a.acceleration.y, a.acceleration.x);                         // Calcul angle statique
    TetaGF = A * TetaG + B * TetaGF;                                            // Passe-bas numérique

    // Traitement Gyroscope
    TetaW = g.gyro.z * Tau / 1000;                                              // Intégration vitesse angulaire
    TetaWF = A * TetaW + B * TetaWF;                                            // Filtrage complémentaire

    // Fusion des signaux
    Teta = TetaWF + TetaGF;                                                     // Estimation angle final

    // Lecture des encodeurs
    TetaMG = encoderL.getCount();
    TetaMD = encoderR.getCount();

    // Calcul des vitesses
    deltaEncodeurMG = ((float)TetaMG - (float)encodeur_precedentMG) / (Te / 1000.0);
    deltaEncodeurMD = ((float)TetaMD - (float)encodeur_precedentMD) / (Te / 1000.0);
    deltaEncodeurMG_Angulaire = deltaEncodeurMG * (2.0 * PI / Nb_de_ticks);
    deltaEncodeurMD_Angulaire = deltaEncodeurMD * (2.0 * PI / Nb_de_ticks);
    deltaMoyenne = (deltaEncodeurMG_Angulaire + deltaEncodeurMD_Angulaire) / 2.0;

    vitesseLineaire = deltaMoyenne * RayonRoue;                                 // Vitesse en m/s
    vitesseLineaireF = AVitesse * vitesseLineaire + BVitesse * vitesseLineaireF;// Filtrage vitesse

    // Asservissement de Vitesse (Boucle externe)
    erreurVitesse = VitesseConsigne - vitesseLineaireF;
    deriveVitesse = erreurVitesse - erreurPrecedentVitesse;
    erreurPrecedentVitesse = erreurVitesse;
    TetaConsigne = kpVitesse * erreurVitesse + kdVitesse * deriveVitesse;       // Consigne d'angle générée par le correcteur de vitesse
    TetaConsigne = constrain(TetaConsigne, -2.0 / 180 * PI, 2.0 / 180 * PI);    // Saturation consigne angle

    // Asservissement de Position/Équilibre (Boucle interne)
    erreurTeta = TetaConsigne - Teta;
    Ec = -kpPosition * erreurTeta + kdPosition * g.gyro.z;                      // Commande générée par le correcteur d'inclinaison

    // Compensation non-linéaire (Frottements)
    if (Ec > 0) Ec += CO1;
    if (Ec < 0) Ec -= CO2;

    // Protection et saturation puissance
    if (Ec > 0.45) Ec = 0.45;
    if (Ec < -0.45) Ec = -0.45;

    // Conversion en rapports cycliques
    dutyCyclePositif = (0.5 + Ec) * 1023;
    dutyCycleNegatif = (0.5 - Ec) * 1023;

    // Commande moteurs
    ledcWrite(MOTGplus, dutyCyclePositif + offsetplusG);
    ledcWrite(MOTDplus, dutyCyclePositif + offsetplusD);
    ledcWrite(MOTGmoins, dutyCycleNegatif + offsetmoinsG);
    ledcWrite(MOTDmoins, dutyCycleNegatif + offsetmoinsD);

    // Archivage données cycle n-1
    encodeur_precedentMG = TetaMG;
    encodeur_precedentMD = TetaMD;
    valeurbatterie = (((3.3 / 4095.0) * analogRead(Batterie) * (R1 + R2)) / R2) + 0.3;

    // Finalisation cycle
    FlagCalcul = 1;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));                         // Cadencement strict
  }
}

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("Gyro_AHM_TOURE");
  
  encoderL.attachHalfQuad(34, 35);
  encoderR.attachHalfQuad(27, 13);

  // Configuration Timers PWM
  ledcSetup(MOTGplus, frequence, resolution);
  ledcSetup(MOTDplus, frequence, resolution);
  ledcSetup(MOTGmoins, frequence, resolution);
  ledcSetup(MOTDmoins, frequence, resolution);

  ledcAttachPin(PWMGplus, MOTGplus);
  ledcAttachPin(PWMDplus, MOTDplus);
  ledcAttachPin(PWMGmoins, MOTGmoins);
  ledcAttachPin(PWMDmoins, MOTDmoins);

  // Initialisation I2C IMU
  if (!mpu.begin()) {
    while (1) { delay(10); }
  }

  // Initialisation des coefficients de filtrage
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;
  AVitesse = 1 / (1 + TauVitesse / Te);
  BVitesse = TauVitesse / Te * A;

  // Création de la tâche de régulation
  xTaskCreate(controle, "controle", 10000, NULL, 10, NULL);
}

// --- Système d'interprétation des commandes Bluetooth ---
void reception(char ch)
{
  static String chaine = "";
  if (ch == '*') {
    int index = chaine.indexOf(' ');
    String commande = (index == -1) ? chaine : chaine.substring(0, index);
    String valeur = (index == -1) ? "" : chaine.substring(index + 1);

    // Ajustement dynamique des paramètres
    if (commande == "Tau") { 
      Tau = valeur.toFloat(); 
      A = 1 / (1 + Tau / Te); 
      B = Tau / Te * A; 
    }

    if (commande == "TauVitesse") { 
      TauVitesse = valeur.toFloat(); 
      AVitesse = 1 / (1 + TauVitesse / Te); 
      BVitesse = TauVitesse / Te * AVitesse; 
    }
    if (commande == "Te") { 
      Te = valeur.toInt(); 
      A = 1 / (1 + Tau / Te); 
      B = Tau / Te * A; 
    }
    
    if (commande == "kpPosition") kpPosition = valeur.toFloat();
    if (commande == "kdPosition") kdPosition = valeur.toFloat();
    if (commande == "kpVitesse") kpVitesse = valeur.toFloat() / 100;
    if (commande == "kdVitesse") kdVitesse = valeur.toFloat() / 1000;
    if (commande == "CO1") CO1 = valeur.toFloat();
    if (commande == "CO2") CO2 = valeur.toFloat();
    
    // Commandes de mouvement
    if (commande == "Z") VitesseConsigne = 0.009;                               // Avancer
    if (commande == "S") VitesseConsigne = -0.008;                              // Reculer
    if (commande == "z" || commande == "s") VitesseConsigne = 0.0;               // Stop
    if (commande == "L") offsetplusD = 800;                                     // Gauche
    if (commande == "l") offsetplusD = 0;
    if (commande == "R") offsetplusG = 800;                                     // Droite
    if (commande == "r") offsetplusG = 0;

    chaine = "";
  } else {
    chaine += ch;
  }
}

// --- Boucle de communication ---
void loop()
{
  while (SerialBT.available() > 0) {
    reception(SerialBT.read());
  }

  // Télémétrie Bluetooth
  if (FlagCalcul == 1) {
    SerialBT.printf("w%f\n*", valeurbatterie);
    SerialBT.printf("b%f\n*", kpVitesse);
    SerialBT.printf("p%f\n*", kdVitesse);
    SerialBT.printf("G%f*", vitesseLineaireF);
    FlagCalcul = 0;
  }
}