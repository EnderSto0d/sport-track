/*
 * my_func.c
 *
 *  Created on: Dec 10, 2025
 *      Author: Marwan
 */

#include "my_func.h"
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float DT_VEILLE = 1.0f / 10.0f;
const float DT_SPORT  = 1.0f / 120.0f;

TSkierState SkierState = SKIING;

/* ===== Filtre IIR Butterworth passe-bas =====================================
 * Ordre      : 2
 * Fs         : 120 Hz (mode SPORT)
 * fc         : 10 Hz (mesuré à -3 dB -> 9.99 Hz)
 * Méthode    : scipy.signal.butter (transformée bilinéaire)
 * Calculés par : script/generate_bode_png.py
 * Forme      : directe II transposée (2 états w[0], w[1] par axe)
 * Date       : Avril 2026
 * Remarque   : ce filtre numérique est COMPLEMENTAIRE du filtre analogique
 *              RC passe-bas 50 Hz (anti-repliement) déjà présent en amont
 *              sur le signal capteur. Il coupe les vibrations > 10 Hz
 *              (chocs carres de ski, bruit mécanique) sans dégrader les
 *              accélérations sportives utiles < 3 Hz.
 * ========================================================================== */
#define FILTRE_ORDRE          2
#define FILTRE_FS_HZ          120.0f
#define FILTRE_FC_HZ          10.0f

/* Zone morte appliquée après filtrage IIR sur l'accélération MPU6050 (en g).
 * Equivalent S4 du rôle de my_accel_ng() côté ADXL : annuler les résidus de
 * bruit/biais pour que la friction de my_simufrict (condition accel == 0)
 * puisse stabiliser la vitesse au repos. */
#define MPU_DEADBAND_G        0.05f

static const float FILTRE_B[3] = {  0.04948996f,  0.09897991f,  0.04948996f };
static const float FILTRE_A[3] = {  1.00000000f, -1.27963242f,  0.47759225f };

/* Filtre IIR forme directe II transposée (ordre 2).
 * Les états w[0] et w[1] sont fournis par l'appelant (un jeu par axe). */
float my_filtre_iir(float x_new, float *p_w0, float *p_w1){
	float l_y = FILTRE_B[0] * x_new + *p_w0;
	*p_w0 = FILTRE_B[1] * x_new - FILTRE_A[1] * l_y + *p_w1;
	*p_w1 = FILTRE_B[2] * x_new - FILTRE_A[2] * l_y;
	return l_y;
}

/* Multiplication de deux quaternions (produit de Hamilton) Retranscrit d'internet */
TQuaternion my_multiplier_quaternions(TQuaternion q1, TQuaternion q2){
	TQuaternion q_resultat;
	q_resultat.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	q_resultat.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	q_resultat.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
	q_resultat.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
	return q_resultat;
}

/* Mise à jour du quaternion d'orientation par intégration de la vitesse angulaire */
TQuaternion my_update_rotation(TQuaternion q, float wx, float wy, float wz, float dt){
	float norme_w = sqrtf(wx*wx + wy*wy + wz*wz);
	if (norme_w == 0.0) return q;

	float angle_rot = norme_w * dt;
	float axe_x = wx / norme_w;
	float axe_y = wy / norme_w;
	float axe_z = wz / norme_w;

	TQuaternion q_delta;
	q_delta.w = cosf(angle_rot / 2.0);
	q_delta.x = axe_x * sinf(angle_rot / 2.0);
	q_delta.y = axe_y * sinf(angle_rot / 2.0);
	q_delta.z = axe_z * sinf(angle_rot / 2.0);

	TQuaternion q_resultat = my_multiplier_quaternions(q, q_delta);

	float norme = sqrtf(q_resultat.w*q_resultat.w + q_resultat.x*q_resultat.x +
					    q_resultat.y*q_resultat.y + q_resultat.z*q_resultat.z);
	q_resultat.w /= norme;
	q_resultat.x /= norme;
	q_resultat.y /= norme;
	q_resultat.z /= norme;

	return q_resultat;
}

/* Rotation d'un vecteur accélération par un quaternion (q * v * q_conjugué) */
void my_rotation_vecteur(TAccel *l_accel, TQuaternion q){
	TQuaternion q_vecteur  = {0.0, l_accel->x, l_accel->y, l_accel->z}; // Vecteur encodé en quaternion pur
	TQuaternion q_conjuge  = {q.w, -q.x, -q.y, -q.z}; // Conjugué du quaternion d'orientation

	TQuaternion q_temp     = my_multiplier_quaternions(q, q_vecteur);
	TQuaternion q_resultat = my_multiplier_quaternions(q_temp, q_conjuge);

	l_accel->x = q_resultat.x;
	l_accel->y = q_resultat.y;
	l_accel->z = q_resultat.z;
}

/* ===== MPU6050 I2C (adresse 0x68, AD0 à GND) ===== */

#define MPU6050_ADDR (0x68 << 1)

/* Sortir le MPU6050 du mode sleep (registre 0x6B) */
static void my_mpu6050_init(I2C_HandleTypeDef *hi2c){
	uint8_t l_data = 0;
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B,
					  I2C_MEMADD_SIZE_8BIT, &l_data, 1, HAL_MAX_DELAY);
}

/* Lecture des registres accéléromètre (0x3B à 0x40) */
static void my_mpu6050_lire_accel(I2C_HandleTypeDef *hi2c,
								   int16_t *ax, int16_t *ay, int16_t *az){
	uint8_t l_buffer[6];
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3B,
					 I2C_MEMADD_SIZE_8BIT, l_buffer, 6, HAL_MAX_DELAY);
	*ax = (int16_t)(l_buffer[0] << 8 | l_buffer[1]);
	*ay = (int16_t)(l_buffer[2] << 8 | l_buffer[3]);
	*az = (int16_t)(l_buffer[4] << 8 | l_buffer[5]);
}

/* Lecture des registres gyroscope (0x43 à 0x48) */
static void my_mpu6050_lire_gyro(I2C_HandleTypeDef *hi2c,
								  int16_t *ox, int16_t *oy, int16_t *oz){
	uint8_t l_buffer[6];
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x43,
					 I2C_MEMADD_SIZE_8BIT, l_buffer, 6, HAL_MAX_DELAY);
	*ox = (int16_t)(l_buffer[0] << 8 | l_buffer[1]);
	*oy = (int16_t)(l_buffer[2] << 8 | l_buffer[3]);
	*oz = (int16_t)(l_buffer[4] << 8 | l_buffer[5]);
}

/* Lecture MPU6050 : accel → rotation quaternion → vecteur accél tourné →
 * filtrage IIR par axe → vitesse. Mesure du temps du bloc filtre via SysTick. */
void my_mpu6050_read(TEntrees *p_entrees){
	static bool b_init = false;
	static TQuaternion q = {1.0, 0.0, 0.0, 0.0}; // Quaternion identité (aucune rotation initiale)

	/* Etats persistants du filtre IIR — un couple (w0,w1) par axe */
	static float l_w0_x = 0.0f, l_w1_x = 0.0f;
	static float l_w0_y = 0.0f, l_w1_y = 0.0f;
	static float l_w0_z = 0.0f, l_w1_z = 0.0f;

	if (!b_init){
		my_mpu6050_init(p_entrees->addr_hi2c1);
		/* Mesure du temps d'exécution : le Cortex-M0+ (ARMv6-M) ne possède
		 * PAS de compteur de cycles DWT (pas de DWT->CYCCNT, cf. TRM ARM
		 * DDI 0484C : le DWT du M0+ se limite aux points d'arrêt/watchpoints).
		 * On utilise donc le SysTick : décompteur 24 bits déjà cadencé à
		 * HCLK = 16 MHz (HSI, sans PLL) par HAL_Init() et rechargé toutes
		 * les 1 ms (LOAD = 15999) — aucune initialisation supplémentaire requise.
		 * Budget cible : 8333 µs par itération en mode SPORT (120 Hz). */
		b_init = true;
	}

	// 1. Lire ax, ay, az → initialiser TAccel (plage ±2g : 16384 LSB/g)
	int16_t ax, ay, az;
	my_mpu6050_lire_accel(p_entrees->addr_hi2c1, &ax, &ay, &az);
	TAccel Acceleration;
	Acceleration.x = ax / 16384.0;
	Acceleration.y = ay / 16384.0;
	Acceleration.z = az / 16384.0;

	// 2. Lire wx, wy, wz → convertir °/s en rad/s → appel de my_update_rotation
	int16_t ox, oy, oz;
	my_mpu6050_lire_gyro(p_entrees->addr_hi2c1, &ox, &oy, &oz);
	float wx = (ox / 131.0) * M_PI / 180.0; // Conversion °/s → rad/s
	float wy = (oy / 131.0) * M_PI / 180.0;
	float wz = (oz / 131.0) * M_PI / 180.0;

	float dt = (g_mode == SPORT) ? DT_SPORT : DT_VEILLE; // Période selon le mode actuel
	q = my_update_rotation(q, wx, wy, wz, dt);

	// 3. Appel de my_rotation_vecteur avec le quaternion mis à jour
	my_rotation_vecteur(&Acceleration, q);

	/* 3bis. Compensation de la gravité : dans le repère monde (après rotation
	 * par le quaternion), la pesanteur est portée par Z et vaut +1 g. On la
	 * retranche AVANT le filtre (gain DC = 1, donc équivalent en régime
	 * établi mais sans transitoire de démarrage) pour ne garder que
	 * l'accélération propre du sportif — sinon my_getspeed intégrerait 1 g
	 * en continu, soit une dérive d'environ +35 km/h par seconde. */
	Acceleration.z -= 1.0f;

	/* 4. Filtrage IIR Butterworth passe-bas sur chaque axe
	 *    Mesure du temps d'exécution via SysTick pour validation grille EI.
	 *    Horloge cœur = 16 MHz (HSI, sans PLL) → 1 cycle = 62,5 ns.
	 *    Budget à 120 Hz = 8333 µs ; ordre de grandeur attendu pour 3 appels IIR ordre 2 ≈ 60-90 µs (virgule flottante logicielle, pas de FPU). */
	uint32_t l_tick_start = SysTick->VAL; // Décompteur SysTick (16 MHz)

	Acceleration.x = my_filtre_iir(Acceleration.x, &l_w0_x, &l_w1_x);
	Acceleration.y = my_filtre_iir(Acceleration.y, &l_w0_y, &l_w1_y);
	Acceleration.z = my_filtre_iir(Acceleration.z, &l_w0_z, &l_w1_z);

	uint32_t l_tick_end = SysTick->VAL;
	/* SysTick DECOMPTE : cycles = start - end, corrigé d'un éventuel
	 * rechargement (période 1 ms >> durée mesurée). */
	uint32_t l_cycles_iir = (l_tick_start >= l_tick_end)
			? (l_tick_start - l_tick_end)
			: (l_tick_start + SysTick->LOAD + 1u - l_tick_end);
	(void)l_cycles_iir; // Utile uniquement si DEBUG_TIMING activé

	/* DEBUG_TIMING — à décommenter pour relever le temps d'exécution du filtre.
	 * Occupation = (temps_us / 8333) * 100 en mode SPORT.
	 * // printf("Filtre IIR: %lu cycles = %.1f us (%.2f %% du budget 8333 us)\r\n",
	 * //     (unsigned long)l_cycles_iir,
	 * //     (float)l_cycles_iir / 16.0f,
	 * //     ((float)l_cycles_iir / 16.0f) / 83.33f);
	 */

	/* DEBUG_FILTRE — pour le test S4-3 avec excitation REELLE (pot vibrant,
	 * haut-parleur + GBF ou tapotements au métronome) : déclarer
	 * « TAccel l_brut = Acceleration; » juste AVANT les 3 appels
	 * my_filtre_iir, puis décommenter le printf ci-dessous pour sortir
	 * brut/filtré sur USART2 et comparer les amplitudes sous LabVIEW
	 * (même banc que le Bode du filtre RC en S3).
	 * // printf("F: %.3f %.3f\r\n", l_brut.z, Acceleration.z);
	 */

	/* 5. Zone morte : annule les résidus (bruit capteur, fuite de gravité
	 * due à la dérive du quaternion) pour permettre à my_simufrict
	 * d'appliquer la friction et de ramener la vitesse à zéro au repos. */
	if (my_fabs(Acceleration.x) < MPU_DEADBAND_G) Acceleration.x = 0.0f;
	if (my_fabs(Acceleration.y) < MPU_DEADBAND_G) Acceleration.y = 0.0f;
	if (my_fabs(Acceleration.z) < MPU_DEADBAND_G) Acceleration.z = 0.0f;

	my_getspeed(Acceleration);
}

/* ===== Bluetooth HC-05 (USART1, PA9=TX / PA10=RX, 9600 bauds) ============
 * Envoie périodiquement une trame texte courte avec :
 *   V : vitesse vectorielle courante (km/h)
 *   A : accélération maximale (valeur absolue d'un axe, en g)
 * Format fixe 5 caractères + 1 décimale -> trame de longueur constante, plus
 * simple à décoder côté application Android. */
#define BT_TAILLE_TRAME 32

void my_bt_send(TEntrees *p_entrees, TVariables *p_variables){
	if (p_entrees == NULL || p_entrees->addr_huart1 == NULL || p_variables == NULL){
		return;
	}

	/* Sélection de la vitesse la plus représentative selon le mode courant */
	float l_vitesse = (g_mode == SPORT)
						? p_variables->sport_speedvector
						: p_variables->veille_speedvector;

	/* Accélération max observée sur un axe (valeur absolue, g) */
	float l_accel_max = my_fabs(p_variables->act_accel.x);
	if (my_fabs(p_variables->act_accel.y) > l_accel_max){
		l_accel_max = my_fabs(p_variables->act_accel.y);
	}
	if (my_fabs(p_variables->act_accel.z) > l_accel_max){
		l_accel_max = my_fabs(p_variables->act_accel.z);
	}

	char l_trame[BT_TAILLE_TRAME];
	int l_len = snprintf(l_trame, BT_TAILLE_TRAME,
						 "V:%05.1f A:%05.1f\r\n", l_vitesse, l_accel_max);
	if (l_len <= 0){
		return;
	}
	if (l_len >= BT_TAILLE_TRAME){
		l_len = BT_TAILLE_TRAME - 1; // Sécurité au cas où snprintf tronque
	}

	HAL_UART_Transmit(p_entrees->addr_huart1,
					  (uint8_t *)l_trame, (uint16_t)l_len, 100);
}

/* Callback TIM3 : déclenche la lecture MPU6050 à 10 Hz (veille) ou 120 Hz (sport).
 * Gère en plus un compteur Bluetooth pour lever g_bt_ready toutes les 500 ms
 * — l'envoi effectif est fait par la boucle principale via my_bt_send(). */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static uint16_t bt_counter = 0; // Compteur de ticks TIM3 pour cadencer le Bluetooth

	if (htim == g_entrees.addr_htim3){ // Vérification que l'interruption vient bien de TIM3
		if (g_entrees.addr_hi2c1 != NULL){
			my_mpu6050_read(&g_entrees);
		}

		/* Seuil d'envoi = 500 ms exprimé en ticks TIM3 (dépend du mode) :
		 *   - mode SPORT  : 120 Hz -> 60 ticks
		 *   - mode VEILLE :  10 Hz -> 5 ticks */
		uint16_t l_seuil = (g_mode == SPORT) ? 60 : 5;
		bt_counter++;
		if (bt_counter >= l_seuil){
			bt_counter = 0;
			g_bt_ready = true;
		}
	}
}


float my_fabs(float value){
	if (value < 0)
		return -value;
	else
		return value;
}

void my_initbuffer_veille(TVariables *variables){
	variables->buffer_veille.position_ecriture = 0;

	for (int i = 0; i < SIZE_BUFFER_VEILLE; i++){
		variables->buffer_veille.donnees[i].x = 0.0;
		variables->buffer_veille.donnees[i].y = 0.0;
		variables->buffer_veille.donnees[i].z = 0.0;
	}
}

void my_initbuffer_sport_inst(TVariables *variables){
	variables->buffer_sport_inst.position_ecriture = 0;

	for (int i = 0; i < SIZE_BUFFER_SPORT_INST; i++){
		variables->buffer_sport_inst.donnees[i].x = 0.0;
		variables->buffer_sport_inst.donnees[i].y = 0.0;
		variables->buffer_sport_inst.donnees[i].z = 0.0;
	}
}

void my_initbuffer_sport_moy(TVariables *variables){
	variables->buffer_sport_moy.position_ecriture = 0;

	for (int i = 0; i < SIZE_BUFFER_SPORT_MOY; i++){
		variables->buffer_sport_moy.donnees[i].x = 0.0;
		variables->buffer_sport_moy.donnees[i].y = 0.0;
		variables->buffer_sport_moy.donnees[i].z = 0.0;
	}
}

/* Initialisation des buffers */
void my_init_buffer(TVariables *variables){
	my_initbuffer_veille(variables);
	my_initbuffer_sport_inst(variables);
	my_initbuffer_sport_moy(variables);
}

/* Initialisation des variables */
void my_variables_init(TVariables *variables){
	variables->b_isTim3On = false; 
	variables->tim3_count = 0;
	variables->synchro_done = false;
	variables->base_accel = (TAccel){0.0, 0.0, 0.0};
	variables->veille_speedvector = 0.0;
	variables->sport_speedvector = 0.0;
	variables->buzzer_timer = (TTimer){0, 0, false};
	variables->speed_max_vector = 0.0;
	variables->act_accel = (TAccel){0.0, 0.0, 0.0};
	variables->speed_inst = (TSpeed){0.0, 0.0, 0.0};
	#ifdef LABVIEWTEST
		variables->labview_timer = (TTimer){DELAY_LABVIEW, 0, false};
		variables->labview_speedvector = 0.0;
	#endif
	my_init_buffer(variables);
}


void my_gestion_buffer_veille(TSpeed speed, TVariables *variables){
	variables->buffer_veille.donnees[variables->buffer_veille.position_ecriture] = speed;

	variables->buffer_veille.position_ecriture++;
	if (variables->buffer_veille.position_ecriture >= SIZE_BUFFER_VEILLE){
		variables->buffer_veille.position_ecriture = 0;
	}
	
}

void my_gestion_buffer_sport_inst(TSpeed speed, TVariables *variables){
	variables->buffer_sport_inst.donnees[variables->buffer_sport_inst.position_ecriture] = speed;

	variables->buffer_sport_inst.position_ecriture++;
	if (variables->buffer_sport_inst.position_ecriture >= SIZE_BUFFER_SPORT_INST){
		variables->buffer_sport_inst.position_ecriture = 0;
	}
	
}

void my_gestion_buffer_sport_moy(TSpeed speed, TVariables *variables){
	variables->buffer_sport_moy.donnees[variables->buffer_sport_moy.position_ecriture] = speed;

	variables->buffer_sport_moy.position_ecriture++;
	if (variables->buffer_sport_moy.position_ecriture >= SIZE_BUFFER_SPORT_MOY){
		variables->buffer_sport_moy.position_ecriture = 0;
	}
}

void my_buffer_sport_moy(TVariables *variables){
	TSpeed speed_moy = {0.0, 0.0, 0.0};
	BufferSportMoyInst save_buffer_inst = variables->buffer_sport_inst;

	for (int i = 0; i < SIZE_BUFFER_SPORT_INST; i++){
		speed_moy.x += save_buffer_inst.donnees[i].x;
		speed_moy.y += save_buffer_inst.donnees[i].y;
		speed_moy.z += save_buffer_inst.donnees[i].z;
	}

	speed_moy.x = speed_moy.x / SIZE_BUFFER_SPORT_INST;
	speed_moy.y = speed_moy.y / SIZE_BUFFER_SPORT_INST;
	speed_moy.z = speed_moy.z / SIZE_BUFFER_SPORT_INST;

	my_gestion_buffer_sport_moy(speed_moy, variables);
}

void my_moytot_sport(TVariables *variables){
	TSpeed speed_moytot = {0.0, 0.0, 0.0};
	BufferSportMoyTot save_buffer_moy = variables->buffer_sport_moy;

	for (int i = 0; i < SIZE_BUFFER_SPORT_MOY; i++){
		speed_moytot.x += save_buffer_moy.donnees[i].x;
		speed_moytot.y += save_buffer_moy.donnees[i].y;
		speed_moytot.z += save_buffer_moy.donnees[i].z;
	}

	speed_moytot.x = speed_moytot.x / SIZE_BUFFER_SPORT_MOY;
	speed_moytot.y = speed_moytot.y / SIZE_BUFFER_SPORT_MOY;
	speed_moytot.z = speed_moytot.z / SIZE_BUFFER_SPORT_MOY;

	variables->speed_moytot_axe = speed_moytot;
}


void my_accelerometerinit(TVariables *variables){
TAccel base_accel = {0.0, 0.0, 0.0};
BufferVeille save_buffer_accelinit = variables->buffer_veille;

if (!variables->synchro_done){
		// Calcul de l'accélération de base
		for (int i = 0; i < SIZE_BUFFER_VEILLE; i++){
			base_accel.x += save_buffer_accelinit.donnees[i].x;
			base_accel.y += save_buffer_accelinit.donnees[i].y;
			base_accel.z += save_buffer_accelinit.donnees[i].z;
		}
		base_accel.x = base_accel.x / SIZE_BUFFER_VEILLE;
		base_accel.y = base_accel.y / SIZE_BUFFER_VEILLE;
		base_accel.z = base_accel.z / SIZE_BUFFER_VEILLE;
		variables->base_accel = base_accel;
		variables->synchro_done = true;
		#ifdef MODEDEBUG
			printf("Synchronisation terminée. Accélération de base : x = %f, y = %f, z = %f\r\n", base_accel.x, base_accel.y, base_accel.z);
		#endif
	}
}

void my_buffer_veille(TVariables *variables){
	TSpeed speed_moy = {0.0, 0.0, 0.0};
	BufferVeille save_buffer_veille = variables->buffer_veille;
	
	for (int i = 0; i < SIZE_BUFFER_VEILLE; i++){
		speed_moy.x += save_buffer_veille.donnees[i].x;
		speed_moy.y += save_buffer_veille.donnees[i].y;
		speed_moy.z += save_buffer_veille.donnees[i].z;
	}
	speed_moy.x = speed_moy.x / SIZE_BUFFER_VEILLE;
	speed_moy.y = speed_moy.y / SIZE_BUFFER_VEILLE;
	speed_moy.z = speed_moy.z / SIZE_BUFFER_VEILLE;
	variables->speed_veille_moy_axe = speed_moy;

	// Debug pour afficher le buffer veille complet
	//   printf("Buffer veille: \r\n "); 
    //   for(int i = 0; i < SIZE_BUFFER_VEILLE; i++){
    //     printf("(%f,%f,%f) \r\n ", save_buffer_veille.donnees[i].x, save_buffer_veille.donnees[i].y, save_buffer_veille.donnees[i].z);
    //   }
    //   printf("\r\n");
	// Debug pour afficher la vitesse moyenne calculée
	// printf("Valeur moyenne buffer veille calculée : \r\n");
	// printf("x : %f y : %f z : %f \r\n", speed_moy.x, speed_moy.y, speed_moy.z); // Debug pour afficher la vitesse instantanée en x, y et z
}

void my_calc_veillespeedvector(void){
	g_variables.veille_speedvector = my_fabs(g_variables.speed_veille_moy_axe.x) + my_fabs(g_variables.speed_veille_moy_axe.y) + my_fabs(g_variables.speed_veille_moy_axe.z);
	if (g_variables.veille_speedvector > g_variables.speed_max_vector){
		g_variables.speed_max_vector = g_variables.veille_speedvector;
		my_gestion_buzzer(200, 0); // Buzzer mode 1 pendant 200 ms
	}
}

void my_calc_sportspeedvector(void){
	g_variables.sport_speedvector = my_fabs(g_variables.speed_moytot_axe.x) + my_fabs(g_variables.speed_moytot_axe.y) + my_fabs(g_variables.speed_moytot_axe.z);
	if (g_variables.sport_speedvector > g_variables.speed_max_vector){
		g_variables.speed_max_vector = g_variables.sport_speedvector;
		my_gestion_buzzer(200, 0); // Buzzer mode 1 pendant 200 ms
	}
}

void my_veille_to_sportbuffer(void){
	TSpeed tempspeed_veille = g_variables.speed_veille_moy_axe;
	
	g_variables.sport_speedvector = g_variables.veille_speedvector;
	g_variables.speed_moytot_axe = g_variables.speed_veille_moy_axe;  // Transmet la vitesse de veille au sport
	for (int i = 0; i < SIZE_BUFFER_SPORT_MOY; i++){
		g_variables.buffer_sport_moy.donnees[i] = tempspeed_veille;
	}
	for (int i = 0; i < SIZE_BUFFER_SPORT_INST; i++){
		g_variables.buffer_sport_inst.donnees[i] = tempspeed_veille;
	}
	my_initbuffer_veille(&g_variables);
}

void my_sport_to_veillebuffer(void){
	TSpeed tempspeed_sport = g_variables.speed_moytot_axe;

	g_variables.veille_speedvector = g_variables.sport_speedvector;
	g_variables.speed_veille_moy_axe = g_variables.speed_moytot_axe;  // Transmet la vitesse du sport à veille
	for (int i = 0; i < SIZE_BUFFER_VEILLE; i++){
		g_variables.buffer_veille.donnees[i] = tempspeed_sport;
	}
	my_initbuffer_sport_inst(&g_variables);
	my_initbuffer_sport_moy(&g_variables);
}

void my_debug_print_mode(TEntrees *p_entrees, TMode *mode){
	#ifdef MODEDEBUG
		printf("Valeur Prescale %lu, Period %lu \r\n", p_entrees->addr_htim3->Instance->PSC, p_entrees->addr_htim3->Instance->ARR);// Debug pour afficher les valeurs de prescale et period du timer3
		printf("Mode actuel : %i \r\n", *mode); // Debug pour afficher le mode actuel
	#endif
}

/* Demarrage du mode */
void my_start_mode(TEntrees *p_entrees, TMode *mode, TVariables *variables){

		switch (*mode)
		{
		case VEILLE:
			p_entrees->addr_htim3->Instance->ARR = 4000; // On règle la periode pour avoir une frequence de 10 Hz (On doit avoir un prescale de 100)
			if (!variables->b_isTim3On){
				HAL_TIM_Base_Start_IT(p_entrees->addr_htim3); // Demarrage du timer3
				variables->b_isTim3On = true; // Variable permettant de savoir si le timer3 est actif
				my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			}
			break;
		case VEILLE_ATT:
			if (variables->b_isTim3On){
				HAL_TIM_Base_Stop_IT(p_entrees->addr_htim3); // Arret du timer3
				variables->b_isTim3On = false; // Desactivation du timer3
				my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			}
			break;
		case SYNCHRO:
			p_entrees->addr_htim3->Instance->ARR = 4000; // On règle la periode pour avoir une frequence de 10 Hz (On doit avoir un prescale de 100)
			if (!variables->b_isTim3On){
				HAL_TIM_Base_Start_IT(p_entrees->addr_htim3); // Demarrage du timer3
				variables->b_isTim3On = true; // Variable permettant de savoir si le timer3 est actif
				my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			}
			break;
		case SYNCHRO_ATT:
			if (variables->b_isTim3On){
				HAL_TIM_Base_Stop_IT(p_entrees->addr_htim3); // Arret du timer3
				variables->b_isTim3On = false; // Desactivation du timer3
				my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			}
			break;
		case SPORT:
			p_entrees->addr_htim3->Instance->ARR = 333; // On règle la periode pour avoir une frequence de 120 Hz (On doit avoir un prescale de 100)
			if (!variables->b_isTim3On){ 
				HAL_TIM_Base_Start_IT(p_entrees->addr_htim3); // Demarrage du timer3
				variables->b_isTim3On = true;  
				my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			}
			break;
		case SPORT_ATT_ETEINT:
			if (variables->b_isTim3On){
				HAL_TIM_Base_Stop_IT(p_entrees->addr_htim3); // Arret du timer3
				variables->b_isTim3On = false; // Desactivation du timer3
				my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			}
			break;
		case SPORT_ATT_VEILLE:
			if (variables->b_isTim3On){
				HAL_TIM_Base_Stop_IT(p_entrees->addr_htim3); // Arret du timer3
				variables->b_isTim3On = false; // Desactivation du timer3
				my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			}
			break;
		case ETEINT:
			if (variables->b_isTim3On){
				HAL_TIM_Base_Stop_IT(p_entrees->addr_htim3); // Arret du timer3
				variables->b_isTim3On = false; // Desactivation du timer3
				my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			}
			break;
		
		default:
			break;
		}

}

void my_accel_ng(float Accel, float *baseaccel, float *Acceleration){
	if ((my_fabs(Accel) >= (my_fabs(*baseaccel) * (1 - COEF_SECU_ACCELBASE) - FLAT_ADD_SECU_ACCEL)) 
	&& (my_fabs(Accel) <= (my_fabs(*baseaccel) * (1 + COEF_SECU_ACCELBASE) + FLAT_ADD_SECU_ACCEL))){
		*Acceleration = 0;
	} else {
		*Acceleration = Accel - *baseaccel;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	float zero[3] = {ZERO_X, ZERO_Y, ZERO_Z};
	float Accel = 0;
	int start_Falling = 0;
	int Landed = 0;
	TAccel Acceleration;
	uint16_t l_ADCvalue[3]; // Variable locale pour stocker directement les valeurs de l'ADC

	for (int i = 0; i < 3; i++){
	    l_ADCvalue[i] = ADCvalue[i];
	}
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	for (int i = 0; i < 3; i++){
		#ifdef LABVIEWTEST
			g_variables.tension_convertie[i] = (float)(l_ADCvalue[i]*3300/4096.0); // Conversion de la valeur brute de l'ADC en mV pour LabVIEW
		#endif
		Accel = (float)(((l_ADCvalue[i]*3300/4096.0)- zero[i]) / SENSITIVITY); // Version en g de la valeur reçue de l'ADC
		//printf("%c : %f ", 'x' + i,(float)(((l_ADCvalue[i]*3300/4096.0)- zero[i]) / SENSITIVITY)); // Debug pour voir les valeur d'accel convertit ici directement
		//printf("%c : %f ", 'x' + i,(float)(((l_ADCvalue[i]*3300/4096.0)))); // Debug pour afficher la valeur brute de l'ADC en Volt. Ce qui nous a permis de recalibrer l'accéléromètre
		if (i == 0){
			if (g_variables.synchro_done)
				my_accel_ng(Accel, &g_variables.base_accel.x, &Acceleration.x);
			else
				Acceleration.x = Accel;
			g_variables.act_accel.x = Acceleration.x;
		} else if (i == 1){
			if (g_variables.synchro_done)
				my_accel_ng(Accel, &g_variables.base_accel.y, &Acceleration.y);
			else
				Acceleration.y = Accel;
			g_variables.act_accel.y = Acceleration.y;
		} else if (i == 2){
			if (g_variables.synchro_done)
				my_accel_ng(Accel, &g_variables.base_accel.z, &Acceleration.z);
			else
				Acceleration.z = Accel;
			g_variables.act_accel.z = Acceleration.z;
		}
	}

	//printf("\r\n");
	// printf("Fin Test\r\n");


	  switch (SkierState)
	  {
	  case SKIING:
	  	if (Acceleration.z == 0){
			start_Falling = g_variables.buffer_sport_moy.position_ecriture - 1;

			SkierState = FALLING;
		}
		break;
	  
	  case FALLING:
	  	if ((Acceleration.z > 3) || (Acceleration.z < - 3)){
			Landed = g_variables.buffer_sport_moy.position_ecriture - 1;

			SkierState = SKIING;

			my_jumpheight(start_Falling, Landed);
		}
		break;
	  }

	my_getspeed(Acceleration);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13) {
		switch (g_mode)
		{
		case ETEINT:
			g_mode = ETEINT_ATT;
			my_debug_print_mode(&g_entrees, &g_mode); // Debug pour afficher le mode actuel
			break;
		case VEILLE:
			g_mode = VEILLE_ATT;
			my_debug_print_mode(&g_entrees, &g_mode); // Debug pour afficher le mode actuel
			break;
		case SPORT:
			g_mode = SPORT_ATT_ETEINT;
			my_debug_print_mode(&g_entrees, &g_mode); // Debug pour afficher le mode actuel
			break;
		
		default:
			break;
		}
} else {
      __NOP();
  }
}


/* Fonctions de gestion des entrees */
void lire_entrees(TEntrees *p_entrees){
	p_entrees->b_bouton = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}

void etat_suivant (TMode *mode, TEntrees *p_entrees, TVariables *variables){
	switch (*mode)
	{
	case ETEINT_ATT:
		if (!variables->synchro_done){
			*mode = SYNCHRO;
			my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			variables->tim3_count = 0;
			my_gestion_buzzer(2000, 2); // Buzzer mode 2 pendant 2 seconde (temp de synchronisation)
		} else if (variables->synchro_done && !p_entrees->b_bouton){
			*mode = VEILLE;
			my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			variables->tim3_count = 0;
			my_gestion_buzzer(200, 3); // Buzzer mode 1 pendant 200 ms
		}
		break;
	case VEILLE:
		my_calc_veillespeedvector();
		if (variables->veille_speedvector >= SEUIL_VEILLE_TO_SPORT){
			// printf("Avant Valeur vitesse vectorielle, veille : %f, sport : %f\r\n", g_variables.veille_speedvector, g_variables.sport_speedvector);
			my_veille_to_sportbuffer();
			// printf("Après Valeur vitesse vectorielle, veille : %f, sport : %f\r\n", g_variables.veille_speedvector, g_variables.sport_speedvector);
			*mode = VEILLE_ATT;
			my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			variables->tim3_count = 0;
		}
		break;
	case SYNCHRO:
		if (variables->synchro_done){
			*mode = SYNCHRO_ATT;
			my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			variables->tim3_count = 0;
		}
		break;
	case SYNCHRO_ATT:
		if (!variables->b_isTim3On){
			*mode = VEILLE;
			my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			variables->tim3_count = 0;
			my_gestion_buzzer(200, 3); // Buzzer mode 2 pendant 200 ms
		}
		break;
	case VEILLE_ATT:
		if (!p_entrees->b_bouton){
			*mode = SPORT;
			my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			variables->tim3_count = 0;
			my_gestion_buzzer(200, 3); // Buzzer mode 2 pendant 200 ms
		}
		break;
	case SPORT:
		my_calc_sportspeedvector();
		if (variables->sport_speedvector <= SEUIL_SPORT_TO_VEILLE){
			*mode = SPORT_ATT_VEILLE;
			my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			variables->tim3_count = 0;
		}
		break;	
	case SPORT_ATT_ETEINT:
		if (!p_entrees->b_bouton){
			*mode = ETEINT;
			my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			variables->tim3_count = 0;
			my_gestion_buzzer(200, 3); // Buzzer mode 2 pendant 200 ms
		}
		break;
	case SPORT_ATT_VEILLE:
		if (!variables->b_isTim3On){
			// printf("Avant Valeur vitesse vectorielle, veille : %f, sport : %f\r\n", g_variables.veille_speedvector, g_variables.sport_speedvector);
			my_sport_to_veillebuffer();
			// printf("Après Valeur vitesse vectorielle, veille : %f, sport : %f\r\n", g_variables.veille_speedvector, g_variables.sport_speedvector);
			*mode = VEILLE;
			my_debug_print_mode(p_entrees, mode); // Debug pour afficher le mode actuel
			variables->tim3_count = 0;
			my_gestion_buzzer(200, 3); // Buzzer mode 2 pendant 200 ms

		}
		break;
	default:
		break;
	}
}

void piloter_sortie(TMode *mode, TEntrees *p_entrees, TVariables *variables){
	my_start_mode(p_entrees, mode, variables);
}

void Buzzer_Start(void){
	TIM2->CCR2 = 1;
	HAL_TIM_PWM_Start(g_entrees.addr_htim2, TIM_CHANNEL_2);
}

void Buzzer_Stop(void){
	// printf("BUZZER STOPPED\r\n");
	TIM2->CCR2 = 1;
	HAL_TIM_PWM_Stop(g_entrees.addr_htim2, TIM_CHANNEL_2);
}

void my_buzzermode(int mode){
	switch (mode)
	{
		case 0:
			TIM2->PSC = 66666;
			break;
		case 1:
			TIM2->PSC = 16666;
			break;
		case 2:
			TIM2->PSC = 4443;
			break;
		case 3:
			TIM2->PSC = 443;
			break;
		default:
			break;
	}
}

void my_gestion_buzzer(int duration_ms, int mode){
	// printf("BUZZER CALLED\r\n");
	g_variables.buzzer_timer.timer_ms = duration_ms;
	g_variables.buzzer_timer.timer_count = 0;
	my_buzzermode(mode);
	Buzzer_Start();
	g_variables.buzzer_timer.b_active = true;
}


void my_simufrict(TAccel *accel, TSpeed *speed, float time_interval){
	/* Calcul du coefficient de friction en fonction de la fréquence d'échantillonnage pour se retrouver
	à ne garder que COEF_FRICTION(98 normalement)% par seconde de la valeur de la vitesse si notre acceleration vaut 0*/
	float coeff = (1 - ((1 - COEF_FRICTION) * time_interval)); 

	if (accel->x == 0) {
		speed->x *= coeff - FLAT_SUBSTRACT_FRICTION * time_interval;
	}
	if (accel->y == 0) {
		speed->y *= coeff - FLAT_SUBSTRACT_FRICTION * time_interval;
	}
	if (accel->z == 0) {
		speed->z *= coeff - FLAT_SUBSTRACT_FRICTION * time_interval;
	}

	if (my_fabs(speed->x) < SPEED_SEUIL_FRICTION){
		speed->x = 0;
	}
	if (my_fabs(speed->y) < SPEED_SEUIL_FRICTION){
		speed->y = 0;
	}
	if (my_fabs(speed->z) < SPEED_SEUIL_FRICTION){
		speed->z = 0;
	}
}

// #define MODEACCEL
#define MODESPEED

void my_getspeed(TAccel accel){
	g_variables.speed_inst = (TSpeed){0.0, 0.0, 0.0};
	TSpeed l_accel_synchro = (TSpeed){0.0, 0.0, 0.0};
	int postion_ecriture = 0;
	
	if (g_mode == SPORT){
		postion_ecriture = g_variables.buffer_sport_inst.position_ecriture;
		if (postion_ecriture == 0)
			postion_ecriture = SIZE_BUFFER_SPORT_INST;
		#ifdef MODESPEED
			g_variables.speed_inst.x = g_variables.buffer_sport_inst.donnees[postion_ecriture - 1].x + ((accel.x * 9.81) * (1/120.0)) * 3.6;
			g_variables.speed_inst.y = g_variables.buffer_sport_inst.donnees[postion_ecriture - 1].y + ((accel.y * 9.81) * (1/120.0)) * 3.6;
			g_variables.speed_inst.z = g_variables.buffer_sport_inst.donnees[postion_ecriture - 1].z + ((accel.z * 9.81) * (1/120.0)) * 3.6;
		#endif
		#ifdef MODEACCEL
			g_variables.speed_inst.x = accel.x;
			g_variables.speed_inst.y = accel.y;
			g_variables.speed_inst.z = accel.z;
		#endif
		my_simufrict(&accel, &g_variables.speed_inst, 1/120.0);
		my_gestion_buffer_sport_inst(g_variables.speed_inst, &g_variables);
	} else if (g_mode == VEILLE){
		postion_ecriture = g_variables.buffer_veille.position_ecriture;
		if (postion_ecriture == 0)
			postion_ecriture = SIZE_BUFFER_VEILLE;
		
		#ifdef MODESPEED
			g_variables.speed_inst.x = g_variables.buffer_veille.donnees[postion_ecriture - 1].x + ((accel.x * 9.81) * (0.1)) * 3.6;
			g_variables.speed_inst.y = g_variables.buffer_veille.donnees[postion_ecriture - 1].y + ((accel.y * 9.81) * (0.1)) * 3.6;
			g_variables.speed_inst.z = g_variables.buffer_veille.donnees[postion_ecriture - 1].z + ((accel.z * 9.81) * (0.1)) * 3.6;
		#endif
		#ifdef MODEACCEL
			g_variables.speed_inst.x = accel.x;
			g_variables.speed_inst.y = accel.y;
			g_variables.speed_inst.z = accel.z;
		#endif
		my_simufrict(&accel, &g_variables.speed_inst, 1/10.0);
		my_gestion_buffer_veille(g_variables.speed_inst, &g_variables);
	} else if (g_mode == SYNCHRO){
		postion_ecriture = g_variables.buffer_veille.position_ecriture;
		if (postion_ecriture == 0)
			postion_ecriture = SIZE_BUFFER_VEILLE;

		l_accel_synchro.x = accel.x;
		l_accel_synchro.y = accel.y;
		l_accel_synchro.z = accel.z;
		my_gestion_buffer_veille(l_accel_synchro, &g_variables);
	}
}

void my_jumpheight(int jump_pos, int land_spot){
	// int finFor = land_spot - jump_pos;
	// int PosBuff = 0;
	// float vitesse_moy = 0;
	// float Jump_Height = 0;

	// if (finFor < 0){
	// 	finFor = SIZE_BUFFER_SPORT_MOY + finFor;
	// }
	
	// for (int i = 0; i <= finFor; i++){
	//	PosBuff = i + jump_pos;
	//	if (PosBuff >= SIZE_BUFFER_SPORT_MOY){
	//		PosBuff -= SIZE_BUFFER_SPORT_MOY;
	//	}
	// 	vitesse_moy += g_variables.buffer_sport_moy.donnees[PosBuff].z;
	// }
	
	// vitesse_moy = vitesse_moy/finFor;

	// Jump_Height = vitesse_moy * finFor * (1/120);
	return;
}
