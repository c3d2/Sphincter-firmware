/* mbed specific header files. */
#include "mbed.h"

/* Helper header files. */
#include "DevSPI.h"

/* Component specific header files. */
#include "PowerStep01.h"


#include <EthernetInterface.h>

Serial pc(USBTX, USBRX, 115200);

DigitalOut red(LED3);
DigitalOut green(LED1);
DigitalOut blue(LED2);


uint8_t recvBuffer[1500 + 1];

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
      
      motor->fetch_and_clear_all_status();
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

/* Network ----------------------------------------------------------------------*/

EthernetInterface eth;
UDPSocket socket;
  
void network_init() {
  
  red = 0; blue = 1; green = 0;

  int ret = eth.connect();

  if (ret == 0) {
    red = 0; blue = 0; green = 1;     
    printf("Got IP Address: %s\n", eth.get_ip_address());
  } else {
    printf("Error acquiring IP: %d\n", ret);
    while (1) { 
        red = 1; blue = 0; green = 0;
        wait(0.4);

        red = 0; blue = 0; green = 0;
        wait(0.4);
    }
  }

  socket.open(&eth);
  socket.bind(1337);
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
  //printf("--> Infinite Loop...\r\n");
  //while (true) {
    //open_door();
    //wait_ms(2000);

    //close_door();
    //wait_ms(1000);
    //printf("status=%x\n", motor->read_status_register());
 
  //}

  network_init();


  while (1) {
    SocketAddress address;
    int recvSize = socket.recvfrom(&address, recvBuffer, 1500);

    if (recvSize < 0) {
        printf("UDP receive error: %d\n", recvSize);
        red = 1;
    } else if (recvSize > 0) {
        printf("UDP received packet of size: %d\n", recvSize);
        recvBuffer[recvSize] = 0;
      if (strstr((char*)recvBuffer, "open")) {
        blue = 1;
        green = 1;
        wait(0.3);
        green = 0;
        wait(0.3);
        green = 1;
        wait(0.3);
        blue = 0;
        open_door();
      } else if (strstr((char*)recvBuffer, "close")) {
        blue = 1;
        red = 1;
        wait(0.3);
        red = 0;
        wait(0.3);
        red = 1;
        wait(0.3);
        blue = 0;
        red = 0;
        close_door();
      }
    }
    
    blue = !blue;
    wait(0.2);

    red = 0;
  }
}
