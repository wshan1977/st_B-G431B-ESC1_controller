
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
#include <string.h>
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
     RPM=5010 IPH=1836mA IQ=-1830mA IBUS=250mA VBUS=12V
   IQ is the signed torque-producing current (negative = braking/generating),
   IBUS is estimated from electrical power / bus voltage (no bus shunt). */
#define TELEM_PERIOD_TICKS   100U  /* message period in medium-frequency ticks (~100 ms) */

static char telem_buf[80];
static uint8_t telem_len = 0U;
static uint8_t telem_pos = 0U;

/* Casting simulation — the PC UI sends one-line commands on the same USART2:
     CAST1 : soft throw  (low peak speed, quick run-down)
     CAST2 : hard throw  (max peak speed, long run-down)
     SPD n : remote speed set-point (custom profile streaming from the PC;
             falls back to the dial if no SPD arrives for 1 s)
     DIAL  : leave remote/cast mode, dial control resumes immediately
     STOP  : stop the motor        START : restart at the default speed
   A cast ramps the spool to the profile peak, dwells briefly, then decays the
   speed command exponentially like a real cast spool run-down. The reel brake
   drag then shows up as extra motor torque (P/omega) across the whole speed
   sweep. Outside a cast ("normal mode") the PB12 dial keeps controlling the
   speed; the dial is ignored only while a cast is in progress. */
typedef struct
{
  uint16_t peak_rpm;
  uint16_t ramp_ms;   /* spin-up ramp to the peak */
  uint16_t hold_ms;   /* dwell at the peak before the decay */
  uint16_t decay_k;   /* speed factor per 100 ms step, x1024 (927 = tau 1 s, 974 = tau 2 s) */
} CastProfile_t;

static const CastProfile_t CAST_PROFILES[2] =
{
  { 4000U, 500U, 300U, 927U },  /* CAST1: soft throw, ~1.2 s run-down */
  { 9500U, 500U, 300U, 974U },  /* CAST2: hard throw, ~4 s run-down */
};
#define CAST_END_RPM         1200  /* profile ends here, dial control resumes */
#define CAST_DECAY_STEP_MS   100U
#define SPD_TIMEOUT_TICKS    1000U /* remote (SPD) mode watchdog: 1 s without a
                                      new set-point returns control to the dial */

static uint8_t cast_pending = 0U;  /* 1/2 = requested mode, 0 = none */
static uint8_t cast_state = 0U;    /* 0 idle, 1 ramp+hold, 2 decay */
static uint8_t cast_mode = 0U;
static uint16_t cast_tick = 0U;
static int32_t cast_rpm = 0;
static uint16_t spd_timeout = 0U;  /* >0 while the PC streams SPD set-points */
static int32_t pot_last_rpm = 0;   /* file scope so a finished cast can force a dial re-apply */

static void cast_rx_poll(void)
{
  static char rx_buf[12];
  static uint8_t rx_len = 0U;

  if (0U != LL_USART_IsActiveFlag_ORE(USART2))
  {
    LL_USART_ClearFlag_ORE(USART2);
  }
  while (0U != LL_USART_IsActiveFlag_RXNE_RXFNE(USART2))
  {
    char c = (char)LL_USART_ReceiveData8(USART2);
    if (('\r' == c) || ('\n' == c))
    {
      rx_buf[rx_len] = '\0';
      rx_len = 0U;
      if (0 == strcmp(rx_buf, "CAST1"))
      {
        cast_pending = 1U;
      }
      else if (0 == strcmp(rx_buf, "CAST2"))
      {
        cast_pending = 2U;
      }
      else if (0 == strncmp(rx_buf, "SPD ", 4))
      {
        /* Custom profile streaming: apply the requested rpm and re-arm the
           watchdog. Overrides a running cast; starts the motor if stopped. */
        int32_t rpm = 0;
        const char *s = &rx_buf[4];
        bool ok = ('\0' != *s);

        while (ok && ('\0' != *s))
        {
          if ((*s >= '0') && (*s <= '9'))
          {
            rpm = (rpm * 10) + (int32_t)(*s - '0');
            s++;
          }
          else
          {
            ok = false;
          }
        }
        if (ok)
        {
          if (rpm < POT_SPEED_MIN_RPM) { rpm = POT_SPEED_MIN_RPM; }
          if (rpm > POT_SPEED_MAX_RPM) { rpm = POT_SPEED_MAX_RPM; }
          cast_pending = 0U;
          cast_state = 0U;
          spd_timeout = SPD_TIMEOUT_TICKS;
          if (RUN == MC_GetSTMStateMotor1())
          {
            MC_ProgramSpeedRampMotor1((int16_t)((rpm * SPEED_UNIT) / U_RPM), 100U);
          }
          else if (IDLE == MC_GetSTMStateMotor1())
          {
            MC_ProgramSpeedRampMotor1((int16_t)((rpm * SPEED_UNIT) / U_RPM), 1000U);
            (void)MC_StartMotor1();
          }
          else
          {
            /* transitioning: next SPD line will land in RUN or IDLE */
          }
        }
      }
      else if (0 == strcmp(rx_buf, "DIAL"))
      {
        cast_pending = 0U;
        cast_state = 0U;
        spd_timeout = 0U;
        pot_last_rpm = 0; /* dial speed re-applies on the next pot update */
      }
      else if (0 == strcmp(rx_buf, "STOP"))
      {
        cast_pending = 0U;
        cast_state = 0U;
        spd_timeout = 0U;
        (void)MC_StopMotor1();
      }
      else if (0 == strcmp(rx_buf, "START"))
      {
        if (IDLE == MC_GetSTMStateMotor1())
        {
          MC_ProgramSpeedRampMotor1((int16_t)DEFAULT_TARGET_SPEED_UNIT, 2000U);
          (void)MC_StartMotor1();
        }
      }
      else
      {
        /* unknown command: ignore */
      }
    }
    else if (rx_len < (uint8_t)(sizeof(rx_buf) - 1U))
    {
      rx_buf[rx_len] = c;
      rx_len++;
    }
    else
    {
      rx_len = 0U; /* overlong line: discard */
    }
  }
}

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

  /* Casting simulation: parse UART commands and run the cast profile.
     While a cast is active the dial (potentiometer) is ignored. */
  {
    cast_rx_poll();

    /* Remote (SPD) mode watchdog: if the PC stream stops, return to the dial */
    if (0U != spd_timeout)
    {
      spd_timeout--;
      if (0U == spd_timeout)
      {
        pot_last_rpm = 0;
      }
    }

    MCI_State_t state = MC_GetSTMStateMotor1();
    if ((0U != cast_state) && (RUN != state))
    {
      cast_state = 0U; /* stop/fault aborts the cast */
    }

    if (0U != cast_pending)
    {
      if (RUN == state)
      {
        const CastProfile_t *p = &CAST_PROFILES[cast_pending - 1U];
        cast_mode = cast_pending;
        cast_pending = 0U;
        spd_timeout = 0U; /* a built-in cast overrides remote mode */
        cast_rpm = (int32_t)p->peak_rpm;
        cast_state = 1U;
        cast_tick = 0U;
        MC_ProgramSpeedRampMotor1((int16_t)((cast_rpm * SPEED_UNIT) / U_RPM),
                                  p->ramp_ms);
      }
      else if (IDLE == state)
      {
        /* motor stopped: spin it up first, the cast begins once RUN is reached */
        MC_ProgramSpeedRampMotor1((int16_t)(((int32_t)CAST_END_RPM * SPEED_UNIT) / U_RPM),
                                  1000U);
        (void)MC_StartMotor1();
      }
      else
      {
        /* start/stop transition in progress: retry next tick */
      }
    }
    else if (1U == cast_state)
    {
      const CastProfile_t *p = &CAST_PROFILES[cast_mode - 1U];
      cast_tick++;
      if (cast_tick >= (uint16_t)(p->ramp_ms + p->hold_ms))
      {
        cast_state = 2U;
        cast_tick = 0U;
      }
    }
    else if (2U == cast_state)
    {
      cast_tick++;
      if (cast_tick >= CAST_DECAY_STEP_MS)
      {
        cast_tick = 0U;
        const CastProfile_t *p = &CAST_PROFILES[cast_mode - 1U];
        cast_rpm = (cast_rpm * (int32_t)p->decay_k) >> 10;
        if (cast_rpm <= CAST_END_RPM)
        {
          cast_state = 0U;
          pot_last_rpm = 0; /* cast finished: force the dial speed to re-apply */
        }
        else
        {
          MC_ProgramSpeedRampMotor1((int16_t)((cast_rpm * SPEED_UNIT) / U_RPM),
                                    CAST_DECAY_STEP_MS);
        }
      }
    }
    else
    {
      /* normal mode: dial control below */
    }
  }

  /* Potentiometer speed control: filtered PB12 reading mapped onto
     POT_SPEED_MIN_RPM..POT_SPEED_MAX_RPM, applied while the motor runs. */
  {
    static uint16_t pot_tick = 0U;
    static uint32_t pot_filt = 0U;    /* IIR-filtered raw ADC value (u16 range) */
    static bool pot_filt_init = false;

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

      if ((RUN == MC_GetSTMStateMotor1()) && (0U == cast_state)
          && (0U == cast_pending) && (0U == spd_timeout))
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
        qd_t iqd = MC_GetIqdMotor1();
        int32_t iq_mA = (int32_t)((float)iqd.q * S16A_TO_AMP * 1000.0f);
        uint16_t vbus_V = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
        int32_t power_mW = (int32_t)(MC_GetAveragePowerMotor1_F() * 1000.0f);
        int32_t ibus_mA = (vbus_V > 0U) ? (power_mW / (int32_t)vbus_V) : 0;

        int n = snprintf(telem_buf, sizeof(telem_buf),
                         "RPM=%d IPH=%dmA IQ=%dmA IBUS=%dmA VBUS=%uV\r\n",
                         (int)rpm, (int)iph_mA, (int)iq_mA, (int)ibus_mA,
                         (unsigned int)vbus_V);
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
