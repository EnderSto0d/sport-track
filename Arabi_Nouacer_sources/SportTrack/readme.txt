================================================================================
                            SPORT-TRACK
                    Système Embarqué de Mesure Sportive
================================================================================

Auteurs : Paul NOUACER, Arabi MARWAN
Formation : BUT2 ESE - Semestre 4
Date : Avril 2026
Encadrant : M. MARTIN

================================================================================
                            DESCRIPTION
================================================================================

Sport-Track est un système embarqué basé sur STM32L073RZ permettant de 
mesurer différentes performances sportives :
  - Vitesse linéaire (km/h)
  - Accélération maximale (m/s²)
  - Hauteur de saut (cm)
  - Détection de chocs ou chutes

Le dispositif se porte au niveau de la ceinture et fonctionne en conditions
sportives difficiles (ski, course, etc.).
 	
================================================================================
                        SPÉCIFICATIONS TECHNIQUES
================================================================================

Microcontrôleur : STM32L073RZ (Nucleo Board)
Capteur principal : MPU6050 (accéléromètre + gyroscope 3 axes, I2C 0x68)
Filtre numérique : IIR Butterworth passe-bas ordre 2, fc = 10 Hz (Fs = 120 Hz)
Actionneur : Buzzer KPEG130 (feedback sonore)
Liaison sans fil : Module Bluetooth HC-05 (9600 bauds, USART1)
Alimentation : Batterie 9V rechargeable
Autonomie : ≥ 24 heures
Communication : UART 115200 bauds (debug via USB), Bluetooth 9600 bauds (HC-05)

Fréquences d'acquisition :
  - Mode VEILLE : 10 Hz
  - Mode SPORT : 120 Hz

================================================================================
                        STRUCTURE DU PROJET
================================================================================

SportTrack/
├── Core/
│   ├── Inc/
│   │   ├── main.h                  # Définitions principales
│   │   ├── my_func.h               # Types et prototypes métier
│   │   ├── stm32l0xx_it.h         # Prototypes interruptions
│   │   └── stm32l0xx_hal_conf.h   # Configuration HAL
│   ├── Src/
│   │   ├── main.c                  # Point d'entrée, init périphériques
│   │   ├── my_func.c               # Fonctions métier (buffers, calculs)
│   │   ├── stm32l0xx_it.c         # Gestionnaires d'interruptions
│   │   ├── stm32l0xx_hal_msp.c    # Callbacks MSP
│   │   ├── syscalls.c              # Appels système
│   │   ├── sysmem.c                # Gestion mémoire
│   │   └── system_stm32l0xx.c     # Configuration horloge
│   └── Startup/
│       └── startup_stm32l073rztx.s # Script de démarrage
├── Drivers/                         # Bibliothèques HAL et CMSIS
├── SportTrack.ioc                  # Configuration CubeMX
├── STM32L073RZTX_FLASH.ld         # Script Linker
├── README.txt                      # Ce fichier
└── Documentation/
    ├── Dossier_Conception_Logiciel.pdf
    ├── Dossier_Fabrication.pdf
    └── Portfolio.pdf

================================================================================
                        INSTALLATION & COMPILATION
================================================================================

PRÉREQUIS :
  - STM32CubeIDE version 1.18.0 ou supérieure
  - Cable USB pour Nucleo Board
  - (Optionnel) Terminal série (PuTTY, Tera Term, etc.)

ÉTAPES :
  1. Ouvrir STM32CubeIDE
  2. File → Open Projects from File System
  3. Sélectionner le dossier "SportTrack"
  4. Project → Build All (Ctrl+B)
  5. Connecter la Nucleo via USB
  6. Run → Debug As → STM32 C/C++ Application (ou F11)

TERMINAL SÉRIE (Debug optionnel) :
  - Ouvrir un terminal série (115200 bauds, 8N1)
  - Connexion sur le port COM de la Nucleo
  - Messages affichés en temps réel : modes, vitesses, synchronisation

================================================================================
                        UTILISATION
================================================================================

MODES DE FONCTIONNEMENT :
  - ETEINT : Système en attente
  - SYNCHRO : Calibration accéléromètre (2 secondes)
  - VEILLE : Surveillance basse fréquence (10 Hz)
  - SPORT : Mesures haute précision (120 Hz)

DÉMARRAGE :
  1. Alimenter le système (USB ou batterie 9V)
  2. Le buzzer émet un bip grave (500 ms)
  3. Appuyer sur le bouton bleu (PC13) pour calibrer
  4. Rester IMMOBILE pendant 2 secondes (bip continu)
  5. Bip aigu → calibration terminée, mode VEILLE actif

TRANSITIONS AUTOMATIQUES :
  - Mouvement rapide (>25 km/h) → passage en mode SPORT
  - Immobilité (<10 km/h pendant 10s) → retour en mode VEILLE
  - Appui bouton en SPORT → mode ETEINT

FEEDBACK BUZZER :
  - Bip grave (500 ms) : Démarrage système
  - Bip grave long (2s) : Synchronisation en cours
  - Bip aigu (200 ms) : Changement de mode confirmé
  - Bip aigu court : Nouveau record de vitesse

================================================================================
                        CONFIGURATION MATÉRIELLE
================================================================================

CONNEXIONS MPU6050 (I2C1 — capteur principal S4) :
  VCC → 3.3V (depuis Nucleo)
  GND → GND
  SDA → PB7 (I2C1_SDA)
  SCL → PB6 (I2C1_SCL)
  AD0 → GND (adresse I2C = 0x68)

Registres utilisés :
  - PWR_MGMT_1          (0x6B)  : sortie mode sleep (écriture 0x00)
  - ACCEL_XOUT_H..ZOUT_L (0x3B-0x40) : accéléromètre 3 axes, big-endian
  - GYRO_XOUT_H..ZOUT_L  (0x43-0x48) : gyroscope 3 axes, big-endian

Plages :
  - Accéléromètre : ±2g → 16 384 LSB/g
  - Gyroscope     : ±250°/s → 131 LSB/°/s (converti en rad/s)

BLUETOOTH HC-05 (USART1 — liaison sans fil vers Android) :
  VCC → 5V (ou 3.3V selon module, généralement 5V)
  GND → GND
  TXD (HC-05) → PA10 (USART1_RX STM32) — via diviseur 5V→3.3V si module 5V
  RXD (HC-05) → PA9  (USART1_TX STM32)
  Baudrate : 9600 bauds, 8N1
  Format trame : "V:XX.X A:XX.X\r\n" envoyée toutes les 500 ms

CONNEXIONS HÉRITÉES ADXL335 (mode mixte possible — désormais secondaire) :
  X-out → PA0 (ADC_IN0)
  Y-out → PA1 (ADC_IN1)
  Z-out → PA4 (ADC_IN4)

BUZZER KPEG130 :
  Signal PWM → PA1 (TIM2_CH2) via AOP TL081
  GND → GND

BOUTON UTILISATEUR :
  PC13 (bouton bleu Nucleo)

LED DE DEBUG :
  LD2 → PA5 (clignote à la fréquence d'acquisition)

COMMUNICATION :
  USART2_TX → PA2   (debug USB, 115200 bauds)
  USART2_RX → PA3
  USART1_TX → PA9   (HC-05 Bluetooth, 9600 bauds)
  USART1_RX → PA10

================================================================================
                        PARAMÈTRES CALIBRATION
================================================================================

Ces valeurs sont définies dans my_func.h et peuvent être ajustées :

ACCÉLÉROMÈTRE (mesurées expérimentalement) :
  #define ZERO_X 1290    // Offset X en mV
  #define ZERO_Y 1290    // Offset Y en mV
  #define ZERO_Z 1350    // Offset Z en mV
  #define SENSITIVITY 240 // Sensibilité en mV/g

SEUILS DE TRANSITION :
  #define SEUIL_VEILLE_TO_SPORT 25  // km/h
  #define SEUIL_SPORT_TO_VEILLE 10  // km/h

FRICTION (anti-dérive capteur) :
  #define COEF_FRICTION 0.99
  #define FLAT_SUBSTRACT_FRICTION 0.15  // km/h par seconde

BUFFERS :
  #define SIZE_BUFFER_VEILLE 20      // 2 secondes à 10 Hz
  #define SIZE_BUFFER_SPORT_INST 12  // 0.1 seconde à 120 Hz
  #define SIZE_BUFFER_SPORT_MOY 100  // 10 secondes de moyennes

================================================================================
                        TESTS & VALIDATION
================================================================================

Tests S3 (reconduits en S4) :
  1. Vérification buzzer (4 tonalités)
  2. Communication UART
  3. Calibration accéléromètre
  4. Mesure vitesse marche (3-8 km/h)
  5. Transition VEILLE → SPORT
  6. Transition SPORT → VEILLE
  7. Accélération maximale
  8. Vérification ADC (vs oscilloscope)
  9. Autonomie batterie (≥24h)

Tests ajoutés en S4 :
  S4-1. Lecture I2C MPU6050 (vérification ACK 0x68, valeurs brutes)
  S4-2. Annulation gravité via quaternion (incliné statique → V ≈ 0)
  S4-3. Filtre IIR (atténuation à fc = 10 Hz + mesure temps d'exécution)
  S4-4. Liaison Bluetooth HC-05 (réception trame sur Android)

Test d'intégration global : scénario complet d'utilisation (~15 min)

Voir "Dossier_Fabrication.pdf" pages 15-19 pour les procédures détaillées.

================================================================================
                        DÉPANNAGE
================================================================================

PROBLÈME : Valeurs de vitesse instables ou dérivent
SOLUTION : 
  - Vérifier la calibration (mode SYNCHRO, rester immobile)
  - Ajuster COEF_FRICTION et FLAT_SUBSTRACT_FRICTION dans my_func.h
  - Vérifier connexions accéléromètre (masse commune)

PROBLÈME : Pas de son du buzzer
SOLUTION :
  - Vérifier connexion PWM sur PA1
  - Vérifier alimentation 9V de l'AOP TL081
  - Tester avec my_gestion_buzzer(1000, 3) dans main.c

PROBLÈME : Pas de messages dans le terminal série
SOLUTION :
  - Vérifier baudrate : 115200, 8N1
  - Essayer différents ports COM
  - Activer #define MODEDEBUG dans my_func.h

PROBLÈME : Transitions automatiques trop sensibles
SOLUTION :
  - Augmenter SEUIL_VEILLE_TO_SPORT (actuellement 25 km/h)
  - Diminuer SEUIL_SPORT_TO_VEILLE (actuellement 10 km/h)

PROBLÈME : LED PA5 ne clignote pas
SOLUTION :
  - Vérifier que le Timer3 est actif (modes VEILLE ou SPORT)
  - Vérifier HAL_GPIO_TogglePin() dans HAL_ADC_ConvCpltCallback()

================================================================================
                        ÉVOLUTIONS FUTURES
================================================================================

  - Ajout d'un boîtier 3D dédié
  - Résistance accrue aux chocs et à l'humidité
  - Application mobile pour visualisation en temps réel (trame HC-05 déjà disponible)
  - Calcul précis de la hauteur de saut (fonction my_jumpheight())
  - Mode température avec gestion thermique avancée
  - Stockage des sessions sport en mémoire non volatile
  - Passage à un filtre IIR adaptatif (ordre variable selon mode)

================================================================================
                        CONTACTS & SUPPORT
================================================================================

Pour toute question ou suggestion :
  - Paul NOUACER : paul.nouacer@edu.esiee.fr
  - Arabi MARWAN : arabi.marwan@edu.esiee.fr

Encadrant projet :
  - M. MARTIN : martin@esiee.fr

Documentation complète disponible dans le dossier /Documentation/

================================================================================
                        LICENCE & CRÉDITS
================================================================================

Projet académique réalisé dans le cadre du BUT2 ESE à l'ESIEE Paris.
© 2025 - Tous droits réservés.

Bibliothèques utilisées :
  - STM32 HAL Library (STMicroelectronics)
  - CMSIS (ARM)

Composants :
  - STM32L073RZ (STMicroelectronics)
  - MPU6050 (InvenSense) — capteur principal S4
  - HC-05 (Bluetooth SPP) — liaison sans fil S4
  - ADXL335 (Analog Devices) — capteur hérité S3
  - KPEG130 (Buzzer)
  - TL081 (Texas Instruments)

================================================================================
                        VERSION
================================================================================

Version : 2.0 (Semestre 4 — MPU6050 + IIR + Bluetooth)
Date de release : Avril 2026
Dernière mise à jour : 23/04/2026

================================================================================