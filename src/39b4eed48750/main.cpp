/**
 ******************************************************************************
 * @file    main.cpp
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    April 13th, 2016
 * @brief   mbed simple application for the STMicroelectronics X-NUCLEO-IHM03A1
 *          Motor Control Expansion Board: control of 1 motor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

/* mbed specific header files. */
#include "mbed.h"

/* Helper header files. */
#include "DevSPI.h"

/* Component specific header files. */
#include "PowerStep01.h"

/* Variables -----------------------------------------------------------------*/

/* Initialization parameters of the motor connected to the expansion board. */
/* Current mode. */
powerstep01_init_u_t init =
{
 .cm = {
  .cp = {
  /* common parameters */
  /* .cmVmSelection = */ POWERSTEP01_CM_VM_CURRENT, // enum powerstep01_CmVm_t
  582, // Acceleration rate in step/s2, range 14.55 to 59590 steps/s^2
  582, // Deceleration rate in step/s2, range 14.55 to 59590 steps/s^2
  488, // Maximum speed in step/s, range 15.25 to 15610 steps/s
  0, // Minimum speed in step/s, range 0 to 976.3 steps/s
  POWERSTEP01_LSPD_OPT_OFF, // Low speed optimization bit, enum powerstep01_LspdOpt_t
  244.16, // Full step speed in step/s, range 7.63 to 15625 steps/s
  POWERSTEP01_BOOST_MODE_OFF, // Boost of the amplitude square wave, enum powerstep01_BoostMode_t
  281.25, // Overcurrent threshold settings via enum powerstep01_OcdTh_t
  STEP_MODE_1_16, // Step mode settings via enum motorStepMode_t
  POWERSTEP01_SYNC_SEL_DISABLED, // Synch. Mode settings via enum powerstep01_SyncSel_t
  (POWERSTEP01_ALARM_EN_OVERCURRENT|
   POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN|
   POWERSTEP01_ALARM_EN_THERMAL_WARNING|
   POWERSTEP01_ALARM_EN_UVLO|
   POWERSTEP01_ALARM_EN_STALL_DETECTION|
   POWERSTEP01_ALARM_EN_SW_TURN_ON|
   POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD), // Alarm settings via bitmap enum powerstep01_AlarmEn_t
  POWERSTEP01_IGATE_64mA, // Gate sink/source current via enum powerstep01_Igate_t 
  POWERSTEP01_TBOOST_0ns, // Duration of the overboost phase during gate turn-off via enum powerstep01_Tboost_t
  POWERSTEP01_TCC_500ns, // Controlled current time via enum powerstep01_Tcc_t
  POWERSTEP01_WD_EN_DISABLE, // External clock watchdog, enum powerstep01_WdEn_t  
  POWERSTEP01_TBLANK_375ns, // Duration of the blanking time via enum powerstep01_TBlank_t
  POWERSTEP01_TDT_125ns, // Duration of the dead time via enum powerstep01_Tdt_t
  },
  /* current mode parameters */
  32.12, // Hold torque in mV, range from 7.8mV to 1000 mV
  32.12, // Running torque in mV, range from 7.8mV to 1000 mV 
  32.12, // Acceleration torque in mV, range from 7.8mV to 1000 mV
  32.12, // Deceleration torque in mV, range from 7.8mV to 1000 mV
  POWERSTEP01_TOFF_FAST_8us, //Maximum fast decay time , enum powerstep01_ToffFast_t 
  POWERSTEP01_FAST_STEP_12us, //Maximum fall step time , enum powerstep01_FastStep_t 
  3.0, // Minimum on-time in us, range 0.5us to 64us
  21.0, // Minimum off-time in us, range 0.5us to 64us 
  POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ, // Clock setting , enum powerstep01_ConfigOscMgmt_t
  POWERSTEP01_CONFIG_SW_HARD_STOP, // External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t
  POWERSTEP01_CONFIG_TQ_REG_TVAL_USED, // External torque regulation enabling , enum powerstep01_ConfigEnTqReg_t
  POWERSTEP01_CONFIG_VS_COMP_DISABLE, // Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t 
  POWERSTEP01_CONFIG_OC_SD_DISABLE, // Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t
  POWERSTEP01_CONFIG_UVLOVAL_LOW, // UVLO Threshold via powerstep01_ConfigUvLoVal_t
  POWERSTEP01_CONFIG_VCCVAL_15V, // VCC Val, enum powerstep01_ConfigVccVal_t
  POWERSTEP01_CONFIG_TSW_048us, // Switching period, enum powerstep01_ConfigTsw_t
  POWERSTEP01_CONFIG_PRED_DISABLE // Predictive current enabling , enum powerstep01_ConfigPredEn_t 
}};

/* Motor Control Component. */
PowerStep01 *motor;

/* Functions -----------------------------------------------------------------*/

/**
 * @brief  This is an example of user handler for the flag interrupt.
 * @param  None
 * @retval None
 * @note   If needed, implement it, and then attach and enable it:
 *           + motor->attach_flag_irq(&my_flag_irq_handler);
 *           + motor->enable_flag_irq();
 *         To disable it:
 *           + motor->DisbleFlagIRQ();
 */
void my_flag_irq_handler(void)
{
  /* Set ISR flag. */
  motor->isrFlag = TRUE;
  /* Get the value of the status register. */
  unsigned int statusRegister = motor->get_status();
  printf("    WARNING: \"FLAG\" interrupt triggered.\r\n");
  /* Check SW_F flag: if not set, the SW input is opened */
  if ((statusRegister & POWERSTEP01_STATUS_SW_F ) != 0) {
    printf("    SW closed (connected to ground).\r\n");
  }
  /* Check SW_EN bit */
  if ((statusRegister & POWERSTEP01_STATUS_SW_EVN) == POWERSTEP01_STATUS_SW_EVN) {
    printf("    SW turn_on event.\r\n");
  }
  /* Check Command Error flag: if set, the command received by SPI can't be */
  /* performed. This occurs for instance when a move command is sent to the */
  /* Powerstep01 while it is already running */
  if ((statusRegister & POWERSTEP01_STATUS_CMD_ERROR) == POWERSTEP01_STATUS_CMD_ERROR) {
    printf("    Non-performable command detected.\r\n");
  }  
  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO)==0) {
    printf("    undervoltage lock-out.\r\n"); 
  }  
  /* Check thermal STATUS flags: if  set, the thermal status is not normal */
  if ((statusRegister & POWERSTEP01_STATUS_TH_STATUS)!=0) {
    //thermal status: 1: Warning, 2: Bridge shutdown, 3: Device shutdown
    printf("    Thermal status: %d.\r\n", (statusRegister & POWERSTEP01_STATUS_TH_STATUS)>>11);
  }    
  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & POWERSTEP01_STATUS_OCD)==0) {
    printf("    Overcurrent detection.\r\n"); 
  }
  /* Reset ISR flag. */
  motor->isrFlag = FALSE;
}

/**
 * @brief  This is an example of error handler.
 * @param[in] error Number of the error
 * @retval None
 * @note   If needed, implement it, and then attach it:
 *           + motor->attach_error_handler(&my_error_handler);
 */
void my_error_handler(uint16_t error)
{
  /* Printing to the console. */
  printf("Error %d detected\r\n\n", error);
  
  /* Infinite loop */
  while (true) {
  }    
}

/* Main ----------------------------------------------------------------------*/

int main()
{
  /* Printing to the console. */
  printf("STARTING MAIN PROGRAM\r\n");
  printf("    Reminder:\r\n");
  printf("    The position unit is in agreement to the step mode.\r\n");
  printf("    The speed, acceleration or deceleration unit\r\n");
  printf("    do not depend on the step mode and the step unit is a full step.\r\n");
    
//----- Initialization 
  /* Initializing SPI bus. */
  DevSPI dev_spi(D11, D12, D13);

  /* Initializing Motor Control Component. */
  motor = new PowerStep01(D2, D4, D8, D9, D10, dev_spi);
  if (motor->init(&init) != COMPONENT_OK) {
    exit(EXIT_FAILURE);
  }

  /* Attaching and enabling an interrupt handler. */
  motor->attach_flag_irq(&my_flag_irq_handler);
  motor->enable_flag_irq();
    
  /* Attaching an error handler */
  motor->attach_error_handler(&my_error_handler);

  /* Printing to the console. */
  printf("Motor Control Application Example for 1 Motor\r\n");

//----- move of 16000 steps in the FW direction
  printf("--> Moving forward 16000 steps.\r\n");
  motor->move(StepperMotor::FWD, 16000);

  /* Waiting while the motor is active. */
  motor->wait_while_active();

  /* Wait for 2 seconds */  
  wait_ms(2000);

//----- Go to position -6400
  printf("--> Go to position -6400 steps.\r\n");

  /* Request device to go to position -6400 */
  motor->go_to(-6400);
  
  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Get current position of device and print to the console */
  printf("    Position: %d.\r\n", motor->get_position());
  
  /* Wait for 2 seconds */  
  wait_ms(2000);

//----- Go Home
  /* Printing to the console. */
  printf("--> Go to home position.\r\n");
  
  /* Request device to go to Home */
  motor->go_home();
  
  /* Wait for the motor ends moving */
  motor->wait_while_active();
  
  /* Wait for 2 seconds */
  wait_ms(2000);

//----- run the motor BACKWARD at 400 step/s
  printf("--> run the motor backward at 400 step/s.\r\n");
  motor->run(StepperMotor::BWD,400);

//----- Get parameter example   
  /* Wait for device reaches the targeted speed */
  while ((motor->read_status_register() & POWERSTEP01_STATUS_MOT_STATUS) != POWERSTEP01_STATUS_MOT_STATUS_CONST_SPD);
 
  /* Record the reached speed in step/s and print to the console */
  printf("    Reached Speed: %f step/s.\r\n", motor->get_analog_value(POWERSTEP01_SPEED));

//----- Soft stopped required while running
  printf("--> Soft stop requested.\r\n");
 
  /* Request a soft stop of device and keep the power bridges enabled */
  motor->soft_hiz();

  /* Wait for the motor of device ends moving */  
  motor->wait_while_active();

  /* Wait for 2 seconds */
  wait_ms(2000);

  /* Infinite Loop. */
  printf("--> Infinite Loop...\r\n");
  while (true) {
    /* Request device to go position -6400 */
    motor->go_to(-6400);

    /* Waiting while the motor is active. */
    motor->wait_while_active();

    
  /* Wait for 2 seconds */
  wait_ms(2000);

    /* Request device to go position 6400 */
    motor->go_to(6400);
 
    /* Waiting while the motor is active. */
    motor->wait_while_active();

    
  /* Wait for 2 seconds */
  wait_ms(2000);
  } 
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
