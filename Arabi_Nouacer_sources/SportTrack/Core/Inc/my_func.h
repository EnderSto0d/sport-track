/*
 * my_func.h
 *
 *  Created on: Dec 10, 2025
 *      Author: Marwan
 */

#ifndef INC_MY_FUNC_H_
#define INC_MY_FUNC_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/*Valeur mesurée sur notre accelleromètre personnel. Ces valeurs peuvent dépendre de l'acce*/
// #define ZERO_X 1220 // Ancienne valeur mesurée avec l'oscilloscope
// #define ZERO_Y 1220 // Ancienne valeur mesurée avec l'oscilloscope
// #define ZERO_Z 1270 // Ancienne valeur mesurée avec l'oscilloscope

/*LES DEUX DEFINE NE DOIVENT PAS ÊTRE ACTIVÉS EN MÊME TEMPS */
// #define MODEDEBUG // Décommenter pour activer les messages de debug dans la console.
#define LABVIEWTEST // Décommenter pour activer l'envoi des données vers LabVIEW

#ifdef LABVIEWTEST
	#define DELAY_LABVIEW 100 // Délai en ms entre chaque envoi de données vers LabVIEW

#endif


#define ZERO_X 1290 // Nouvelle valeur mesurée avec l'adc
#define ZERO_Y 1290 // Nouvelle valeur mesurée avec l'adc
#define ZERO_Z 1350 // Nouvelle valeur mesurée avec l'adc
#define SENSITIVITY 240 // Sensibilité de l'accéléromètre en g/mV

#define COEF_SECU_ACCELBASE 0.05 // Coefficient de sécurité pour la base d'accélération
#define FLAT_ADD_SECU_ACCEL 0.05 // Valeur ajoutée/soustraite pour éviter d'avoir une zone morte trop petite du à l'acceleration de base petite
#define COEF_FRICTION 0.99 // Coefficient de friction pour la simulation de la vitesse qui se met sur chaque valeur
#define FLAT_SUBSTRACT_FRICTION 0.15 // Valeur soustraite en plus chaque seconde pour éviter les petites valeurs de vitesses qui trainent trop longtemps (en km/h)
#define SPEED_SEUIL_FRICTION 0.3 // Seuil en dessous duquel la vitesse est considérée comme nulle (en km/h)
#define SEUIL_VEILLE_TO_SPORT 25 // Seuil du vecteur vitesse en mode veille pour passer en mode sport (en km/h)
#define SEUIL_SPORT_TO_VEILLE 10 // Seuil du vecteur vitesse en mode sport pour passer en mode veille (en km/h)

extern const float DT_VEILLE; // Période d'échantillonnage en mode veille (10 Hz)
extern const float DT_SPORT;  // Période d'échantillonnage en mode sport (120 Hz)

#define SIZE_BUFFER_VEILLE 20 // Mode veille =  10 Hz -> ici on stocke 20 valeurs en deux secondes
#define SIZE_BUFFER_SPORT_INST 12 // Mode sport = 120 Hz -> ici on stocke 12 valeurs en un dixième de seconde
#define SIZE_BUFFER_SPORT_MOY 100 // Mode sport = 120 Hz -> ici on stocke la moyenne des valeurs moyennes reçues en un dixième de seconde sur 10 secondes

typedef struct {
	 float x;
	 float y;
	 float z;
}TAccel;

typedef struct {
	 float x;
	 float y;
	 float z;
}TSpeed;

typedef struct {
	float w;
	float x;
	float y;
	float z;
}TQuaternion;


typedef struct { // Durée / Compteur / Etat du buzzer
	int timer_ms; // Durée du buzzer en ms
	int timer_count; // Compteur pour la gestion du buzzer
	bool b_active; // Etat du buzzer (Actif ou non)
}TTimer; 

typedef struct {
    TSpeed donnees[SIZE_BUFFER_VEILLE];
    uint16_t position_ecriture;  // Où on écrit la prochaine donnée
} BufferVeille;

typedef struct {
    TSpeed donnees[SIZE_BUFFER_SPORT_INST];
    uint16_t position_ecriture;  // Où on écrit la prochaine donnée
} BufferSportMoyInst;

typedef struct {
    TSpeed donnees[SIZE_BUFFER_SPORT_MOY];
    uint16_t position_ecriture;  // Où on écrit la prochaine donnée
} BufferSportMoyTot;

typedef struct {
	bool b_isTim3On; // Etat du timer3 (Actif ou non)
	BufferVeille buffer_veille; // Buffer pour stocker les vitesses en mode veille
	TSpeed speed_veille_moy_axe; // Vitesse moyenne calculée en mode veille
	TSpeed speed_moytot_axe; // Vitesse moyenne totale calculée en mode sport
	float veille_speedvector; // Vecteur vitesse combinant les 3 axes en mode veille
	float sport_speedvector; // Vecteur vitesse combinant les 3 axes en mode sport
	BufferSportMoyInst buffer_sport_inst; // Buffer pour stocker les vitesses instantanées en mode sport
	BufferSportMoyTot buffer_sport_moy; // Buffer pour stocker les vitesses moyennes en mode sport
	int tim3_count; // Variable pour qui nous sert a compter le nombre de fois où le tim3 est appellé, et est reinitialisé à chaque ecriture d'un buffer
	bool synchro_done; // Variable pour savoir si la synchro a déjà été faite
	TAccel base_accel; // Valeur de l'accélération de base mesurée lors de la synchronisation
	TTimer buzzer_timer; // Durée du buzzer en ms
	float speed_max_vector; // Vitesse maximale atteinte sur un axe
	TAccel act_accel; // Acceleration actuelle lue par l'ADC
	TSpeed speed_inst; // Vitesse instantanée actuelle calculée
	#ifdef LABVIEWTEST
		TTimer labview_timer; // Timer pour l'envoi des données vers LabVIEW
		float labview_speedvector; // Vecteur vitesse combinant les 3 axes pour LabVIEW
		float tension_convertie[3]; // Tableau pour stocker les tensions converties pour LabVIEW
	#endif
}TVariables;

typedef struct  {
	ADC_HandleTypeDef *addr_hadc; // Adresse de l'ADC
	I2C_HandleTypeDef *addr_hi2c1; // Adresse du I2C1 (MPU6050)
	TIM_HandleTypeDef *addr_htim3; // Adresse du timer3
	TIM_HandleTypeDef *addr_htim2; // Adresse du timer2 (Buzzer)
	UART_HandleTypeDef *addr_huart1; // Adresse de l'USART1 (HC-05 Bluetooth, 9600 bauds)
	bool b_bouton;
}TEntrees;

typedef enum {
	ETEINT = 0,
	ETEINT_ATT,
	SYNCHRO,
	SYNCHRO_ATT,
	VEILLE,
	VEILLE_ATT,
	SPORT, 
	SPORT_ATT_ETEINT,
	SPORT_ATT_VEILLE
}TMode;

typedef enum {
	SKIING = 0,
	FALLING
}TSkierState;


extern uint16_t ADCvalue[3];
extern int start_Falling;
extern int Landed;
extern TEntrees g_entrees;
extern TVariables g_variables;
extern TMode g_mode;
extern TSkierState SkierState;

/* Drapeau levé par le callback TIM3 toutes les 500 ms, remis à false
   par la boucle principale juste avant l'appel de my_bt_send(). */
extern volatile bool g_bt_ready;



void my_start_mode(TEntrees *p_entrees, TMode *mode, TVariables *variables);
void lire_entrees(TEntrees *p_entrees);
void etat_suivant (TMode *mode, TEntrees *p_entrees, TVariables *variables);
void piloter_sortie(TMode *mode, TEntrees *p_entrees, TVariables *variables);
void my_variables_init(TVariables *variables);
void my_init_buffer(TVariables *variables);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void my_debug_print_mode(TEntrees *p_entrees, TMode *mode);
void Buzzer_Start(void);
void Buzzer_Stop(void);
void my_buzzermode(int mode);
void my_getspeed(TAccel accel);
float my_fabs(float value); // Valeur absolue flottante (prototype requis : utilisée par my_bt_send avant sa définition)
void my_jumpheight(int jump_pos, int land_spot);
void my_buffer_sport_moy(TVariables *variables);
void my_buffer_veille(TVariables *variables);
void my_moytot_sport(TVariables *variables);
void my_gestion_buzzer(int duration_ms, int mode);
void my_accelerometerinit(TVariables *variables);

TQuaternion my_multiplier_quaternions(TQuaternion q1, TQuaternion q2);
TQuaternion my_update_rotation(TQuaternion q, float wx, float wy, float wz, float dt);
void my_rotation_vecteur(TAccel *l_accel, TQuaternion q);
void my_mpu6050_read(TEntrees *p_entrees);

/* Filtre IIR numérique Butterworth passe-bas (ordre 2, Fs=120 Hz, fc=10 Hz)
   Coefficients calculés par script/generate_bode_png.py (scipy.signal.butter).
   Filtre distinct du filtre analogique RC passe-bas 50 Hz en amont (anti-repliement). */
float my_filtre_iir(float x_new, float *p_w0, float *p_w1);

/* Envoi de la trame vitesse/accélération via HC-05 (USART1, 9600 bauds). */
void my_bt_send(TEntrees *p_entrees, TVariables *p_variables);


#endif /* INC_MY_FUNC_H_ */


