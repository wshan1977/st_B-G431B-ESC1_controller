
/**
  ******************************************************************************
  * @file    mc_app_hooks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements default motor control app hooks.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup MCAppHooks
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "mc_type.h"
#include "mc_app_hooks.h"
#include "mc_config.h"
#include "mc_api.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"
#include "regular_conversion_manager.h"
#include "esc.h"

/* Speed potentiometer on PB12 (ADC1_IN11) — official POTENTIOMETER_LEVEL pin
   of the B-G431B-ESC1 board. PB12 is left in its reset (analog) state. */
#define POT_SPEED_MIN_RPM    1200
#define POT_SPEED_MAX_RPM    9500
#define POT_UPDATE_TICKS     100U  /* update period in medium-frequency ticks (~100 ms) */
#define POT_DELTA_RPM        100   /* minimum change before a new ramp is programmed */

static RegConv_t PotRegConv =
{
  .regADC       = ADC1,
  .channel      = MC_ADC_CHANNEL_11,
  .samplingTime = LL_ADC_SAMPLINGTIME_47CYCLES_5,
};

/* Telemetry on USART2 (PB3/PB4 = ST-LINK virtual COM port over the board USB),
   115200 8N1 set in MX_USART2_UART_Init(). ASPEP/Motor Pilot no longer starts
   (see MCboot), so the telemetry owns the UART.
   One ASCII line every TELEM_PERIOD_TICKS, e.g.:
     RPM=5010 IPH=1836mA IBUS=250mA VBUS=12V
   IBUS is estimated from electrical power / bus voltage (no bus shunt). */
#define TELEM_PERIOD_TICKS   100U  /* message period in medium-frequency ticks (~100 ms) */

static char telem_buf[64];
static uint8_t telem_len = 0U;
static uint8_t telem_pos = 0U;

/* s16A (0-to-peak) to ampere, from the MC_GetPhaseCurrentAmplitudeMotor1() doc:
   I[A] = s16A * Vdd / (65536 * Rshunt * Aop) */
static const float S16A_TO_AMP = 3.3f / (65536.0f * RSHUNT * AMPLIFICATION_GAIN);

static void telem_uart_init(void)
{
  /* USART2 is already configured by MX_USART2_UART_Init(); only the TX FIFO is
     enabled here (requires UE=0) so the sender can burst several bytes per tick. */
  LL_USART_Disable(USART2);
  LL_USART_EnableFIFO(USART2);
  LL_USART_Enable(USART2);
}

/* Power-on "beep-beep": drives the motor windings as a speaker, same technique
   as the MCSDK ESC beep feature (esc.c). Played once while the drive is IDLE,
   before the auto-start delay expires. Sound only — no rotor alignment. */
#define BEEP_ARR_TONE1       65000U  /* lower tone */
#define BEEP_ARR_TONE2       55000U  /* higher tone */
#define BEEP_DUTY_CMP        3000U

static void beep_tone_start(uint32_t arr)
{
  LL_TIM_SetAutoReload(TIM1, arr);
  LL_TIM_OC_SetCompareCH1(TIM1, BEEP_DUTY_CMP);
  LL_TIM_OC_SetCompareCH2(TIM1, arr);
  LL_TIM_OC_SetCompareCH3(TIM1, arr);
  LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N
                              | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N);
  LL_TIM_EnableAllOutputs(TIM1);
}

static void beep_pwm_restore(uint32_t arr)
{
  LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2
                               | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH1N
                               | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N);
  /* Compare values must be zeroed before re-enabling the channels: the beep
     left CCR2/CCR3 above the restored ARR (high sides 100% on), which would
     short bus voltage across the windings until the motor starts. CCR=0 keeps
     all low sides on (brake), same idle state as esc.c SM_BEEP_4. */
  LL_TIM_OC_SetCompareCH1(TIM1, 0U);
  LL_TIM_OC_SetCompareCH2(TIM1, 0U);
  LL_TIM_OC_SetCompareCH3(TIM1, 0U);
  LL_TIM_SetAutoReload(TIM1, arr);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2
                              | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH1N
                              | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N);
}

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup COMMON_MC
  * @{
  */

/**
 * @defgroup MCAppHooks Motor Control Applicative hooks
 * @brief User defined functions that are called in the Motor Control tasks.
 *
 *
 * @{
 */

/**
 * @brief Hook function called right before the end of the MCboot function.
 *
 *
 *
 */
__weak void MC_APP_BootHook(void)
{
  /* esc_boot(&ESC_M1); */ /* ESC PWM input bypassed: control via Motor Pilot */
/* USER CODE BEGIN BootHook */
  (void)RCM_RegisterRegConv(&PotRegConv);
  telem_uart_init();
/* USER CODE END BootHook */
}

/**
 * @brief Hook function called right after the Medium Frequency Task for Motor 1.
 *
 *
 *
 */
__weak void MC_APP_PostMediumFrequencyHook_M1(void)
{
  /* esc_pwm_control(&ESC_M1); */ /* ESC PWM input bypassed: control via Motor Pilot */
/* USER SECTION BEGIN PostMediumFrequencyHookM1 */
  /* Power-on beep-beep, played once during the IDLE wait before auto-start */
  {
    static uint8_t beep_stage = 0U;   /* 0:tone1 1:gap 2:tone2 3:restore 4:done */
    static uint16_t beep_tick = 0U;
    static uint32_t beep_saved_arr = 0U;
    static const uint16_t beep_dur[4] = {100U, 60U, 140U, 1U}; /* ms per stage */

    if (beep_stage < 4U)
    {
      if (0U == beep_tick)
      {
        switch (beep_stage)
        {
          case 0U:
            beep_saved_arr = LL_TIM_GetAutoReload(TIM1);
            beep_tone_start(BEEP_ARR_TONE1);
            break;
          case 1U:
            LL_TIM_OC_SetCompareCH1(TIM1, 0U); /* silence between beeps */
            break;
          case 2U:
            LL_TIM_SetAutoReload(TIM1, BEEP_ARR_TONE2);
            LL_TIM_OC_SetCompareCH1(TIM1, BEEP_DUTY_CMP);
            break;
          case 3U:
          default:
            beep_pwm_restore(beep_saved_arr);
            break;
        }
      }
      beep_tick++;
      if (beep_tick >= beep_dur[beep_stage])
      {
        beep_stage++;
        beep_tick = 0U;
      }
    }
  }

  /* Standalone auto-start: no PWM input / no USB. Runs at the medium-frequency
     task rate (1 kHz): start ~2 s after power-up, on fault retry after ~3 s. */
  {
    static uint16_t autostart_delay = 2000U;
    static bool autostart_done = false;

    if (autostart_delay > 0U)
    {
      autostart_delay--;
    }
    else if ((false == autostart_done) && (IDLE == MC_GetSTMStateMotor1()))
    {
      MC_ProgramSpeedRampMotor1((int16_t)DEFAULT_TARGET_SPEED_UNIT, 2000U);
      (void)MC_StartMotor1();
      autostart_done = true;
    }
    else if (FAULT_OVER == MC_GetSTMStateMotor1())
    {
      (void)MC_AcknowledgeFaultMotor1();
      autostart_done = false;
      autostart_delay = 3000U;
    }
    else
    {
      /* Nothing to do */
    }
  }

  /* Potentiometer speed control: filtered PB12 reading mapped onto
     POT_SPEED_MIN_RPM..POT_SPEED_MAX_RPM, applied while the motor runs. */
  {
    static uint16_t pot_tick = 0U;
    static uint32_t pot_filt = 0U;    /* IIR-filtered raw ADC value (u16 range) */
    static bool pot_filt_init = false;
    static int32_t pot_last_rpm = 0;

    pot_tick++;
    if (pot_tick >= POT_UPDATE_TICKS)
    {
      pot_tick = 0U;
      uint32_t raw = (uint32_t)RCM_GetRegularConv(&PotRegConv);
      if (false == pot_filt_init)
      {
        pot_filt = raw;
        pot_filt_init = true;
      }
      else
      {
        pot_filt = ((pot_filt * 7U) + raw) >> 3;
      }

      if (RUN == MC_GetSTMStateMotor1())
      {
        int32_t rpm = POT_SPEED_MIN_RPM
                    + (int32_t)((pot_filt * (uint32_t)(POT_SPEED_MAX_RPM - POT_SPEED_MIN_RPM)) >> 16);
        int32_t delta = rpm - pot_last_rpm;
        if ((delta > POT_DELTA_RPM) || (delta < -POT_DELTA_RPM))
        {
          pot_last_rpm = rpm;
          MC_ProgramSpeedRampMotor1((int16_t)((rpm * SPEED_UNIT) / U_RPM), 250U);
        }
      }
    }
  }
  /* Telemetry: non-blocking sender. Every tick the pending message is drained
     into the USART1 TX FIFO; a new line is built every TELEM_PERIOD_TICKS. */
  {
    static uint16_t telem_tick = 0U;

    while ((telem_pos < telem_len) && (0U != LL_USART_IsActiveFlag_TXE_TXFNF(USART2)))
    {
      LL_USART_TransmitData8(USART2, (uint8_t)telem_buf[telem_pos]);
      telem_pos++;
    }

    telem_tick++;
    if (telem_tick >= TELEM_PERIOD_TICKS)
    {
      telem_tick = 0U;
      if (telem_pos >= telem_len) /* previous line fully queued */
      {
        int32_t rpm = ((int32_t)MC_GetMecSpeedAverageMotor1() * U_RPM) / SPEED_UNIT;
        int32_t iph_mA = (int32_t)((float)MC_GetPhaseCurrentAmplitudeMotor1()
                                   * S16A_TO_AMP * 1000.0f);
        uint16_t vbus_V = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
        int32_t power_mW = (int32_t)(MC_GetAveragePowerMotor1_F() * 1000.0f);
        int32_t ibus_mA = (vbus_V > 0U) ? (power_mW / (int32_t)vbus_V) : 0;

        int n = snprintf(telem_buf, sizeof(telem_buf),
                         "RPM=%d IPH=%dmA IBUS=%dmA VBUS=%uV\r\n",
                         (int)rpm, (int)iph_mA, (int)ibus_mA, (unsigned int)vbus_V);
        if (n > 0)
        {
          telem_len = ((size_t)n < sizeof(telem_buf)) ? (uint8_t)n
                                                      : (uint8_t)(sizeof(telem_buf) - 1U);
          telem_pos = 0U;
        }
      }
    }
  }
/* USER SECTION END PostMediumFrequencyHookM1 */
}

/** @} */

/** @} */

/** @} */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
