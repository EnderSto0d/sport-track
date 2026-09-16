/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_func.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  if (g_variables.buzzer_timer.b_active){
    g_variables.buzzer_timer.timer_count++;
    if (g_variables.buzzer_timer.timer_count >= g_variables.buzzer_timer.timer_ms){
      // printf("BUZZER STOPPED\\r\\n");
      Buzzer_Stop();
      g_variables.buzzer_timer.b_active = false;
      g_variables.buzzer_timer.timer_count = 0;
    }
  }
  #ifdef LABVIEWTEST
    if (g_variables.labview_timer.b_active){
      g_variables.labview_timer.timer_count++;
      if (g_variables.labview_timer.timer_count >= g_variables.labview_timer.timer_ms){
        g_variables.labview_timer.b_active = false;
        g_variables.labview_timer.timer_count = 0;
      }
    }
  #endif
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  int position_ecriture = 0;
  // printf("Valeur chrono : %d, Fréquence actuelle Timer : %d \r\n", g_variables.tim3_count, (4000000/g_entrees.addr_htim3->Instance->ARR/g_entrees.addr_htim3->Instance->PSC)); // Debug pour afficher la valeur du chrono
  if (g_mode == SPORT){
    g_variables.tim3_count++;
    if (g_variables.tim3_count >= SIZE_BUFFER_SPORT_INST) { // Fréquence de 120 Hz donc on traite toutes les 1/120 * 12 = 10 ms 
        position_ecriture = g_variables.buffer_sport_inst.position_ecriture;
        if (position_ecriture == 0)
          position_ecriture = SIZE_BUFFER_SPORT_INST;
      g_variables.tim3_count = 0;
      my_buffer_sport_moy(&g_variables);
      my_moytot_sport(&g_variables);
      #ifdef MODEDEBUG
        printf("Buffer sport moy mis à jour. Position écriture : %d\r\n", g_variables.buffer_sport_moy.position_ecriture);
        printf(" x : %f     y : %f      z : %f\r\n", g_variables.buffer_sport_moy.donnees[g_variables.buffer_sport_moy.position_ecriture - 1].x,
        g_variables.buffer_sport_moy.donnees[g_variables.buffer_sport_moy.position_ecriture - 1].y,
        g_variables.buffer_sport_moy.donnees[g_variables.buffer_sport_moy.position_ecriture - 1].z); 
         // Debug pour afficher la vitesse instantanée en x, y et z
        printf("vecteur sport : %f\r\n", g_variables.sport_speedvector);
      #endif

    }
  } else if (g_mode == VEILLE){
    g_variables.tim3_count++;
    if  (g_variables.tim3_count >= SIZE_BUFFER_VEILLE) { // Fréquence de 10 Hz donc on traite toutes les 2s
      g_variables.tim3_count = 0;
      my_buffer_veille(&g_variables);

      // Print du buffer veille complet
      #ifdef MODEDEBUG
        printf("x : %f     y : %f      z : %f \r\n", g_variables.speed_veille_moy_axe.x, g_variables.speed_veille_moy_axe.y, g_variables.speed_veille_moy_axe.z); // Debug pour afficher la vitesse instantanée en x, y et z
      #endif
    }
  } else if (g_mode == SYNCHRO){
    g_variables.tim3_count++;
    if  (g_variables.tim3_count >= SIZE_BUFFER_VEILLE + 20) { // On va venir traiter 4 seconde après le début de la synchro. On prendra les 2 dernière secondes pour calculer l'accélération de base
      g_variables.tim3_count = 0;
      my_accelerometerinit(&g_variables);

      // Print du buffer veille complet
      #ifdef MODEDEBUG
        printf("x : %f     y : %f      z : %f \r\n", g_variables.speed_veille_moy_axe.x, g_variables.speed_veille_moy_axe.y, g_variables.speed_veille_moy_axe.z); // Debug pour afficher la vitesse instantanée en x, y et z
      #endif
    }
  }

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
