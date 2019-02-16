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


Serial pc(USBTX, USBRX, 115200);

/* Variables -----------------------------------------------------------------*/


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

volatile bool has_flag;
void my_set_flag()  {
    has_flag = true;
}

// pur own busy waiting that only polls irq flag...
void wait_while_active() {
  while (motor->is_device_busy()) {
    if (has_flag) {
      my_flag_irq_handler();
      has_flag = false;
      return;
    }

    wait(0.1);
  }
}

void open_door()
{
  
    // Target position. This will be our timeout/fallback
//----- move of 16000 steps in the FW direction
  printf("--> Moving forward 16000 steps.\r\n");
  motor->move(StepperMotor::FWD, 192000);
    /* Waiting while the motor is active. */
    wait_while_active();
    motor->move(StepperMotor::BWD, 13000);
    
    wait_while_active();

  // Move back by quarter rotation
    
//----- Soft stopped required while running
  printf("--> Soft stop requested.\r\n");
 
  /* Request a soft stop of device and keep the power bridges enabled */
  motor->soft_hiz();
}

void close_door()
{
//----- run the motor BACKWARD at 400 step/s
  printf("--> run the motor backward at 400 step/s.\r\n");
  motor->move(StepperMotor::BWD,205000);

  
    /* Waiting while the motor is active. */
    wait_while_active();
  
//----- Soft stopped required while running
  printf("--> Soft stop requested.\r\n");
 
  /* Request a soft stop of device and keep the power bridges enabled */
  motor->soft_hiz();
}

/* Main ----------------------------------------------------------------------*/

int main()
{
//----- Initialization 
  /* Initializing SPI bus. */
  DevSPI dev_spi(D11, D12, D13);
  /* Initializing Motor Control Component. */
  motor = new PowerStep01(D2, D4, D8, D9, D10, dev_spi);
  
  if (motor->init(0) != COMPONENT_OK) {
    exit(EXIT_FAILURE);
  }

  /* Attaching and enabling an interrupt handler. */
  //motor->attach_flag_irq(&my_flag_irq_handler);
  motor->attach_flag_irq(&my_set_flag);
  motor->enable_flag_irq();
    
  /* Attaching an error handler */
  motor->attach_error_handler(&my_error_handler);

  /* Printing to the console. */
  printf("Motor Control Application Example for 1 Motor\r\n");


//----- Get parameter example   
  /* Wait for device reaches the targeted speed */
  //while ((motor->read_status_register() & POWERSTEP01_STATUS_MOT_STATUS) != POWERSTEP01_STATUS_MOT_STATUS_CONST_SPD);
 
  /* Record the reached speed in step/s and print to the console */
  //printf("    Reached Speed: %f step/s.\r\n", motor->get_analog_value(POWERSTEP01_SPEED));


  /* Infinite Loop. */
  printf("--> Infinite Loop...\r\n");
  while (true) {
    open_door();
    wait_ms(2000);

    close_door();
    wait_ms(1000);
    printf("status=%x\n", motor->read_status_register());
 
  } 
}
