#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};      // Allow time for clock to stabilize
		
	GPIO_PORTM_DIR_R = 0b00000000;       					  // Enable PM0 as input 
	GPIO_PORTM_DEN_R = 0b00000010;		
	
	GPIO_PORTM_PUR_R |= 0x02;								  // Enable weak pull up resistor on PM1		
	return;
}

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	
    GPIO_PORTB_ODR_R |= 0x08;             																	

    GPIO_PORTB_DEN_R |= 0x0C;             																
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200; 
    I2C0_MCR_R = I2C_MCR_MFE;                      												
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                     
                                       					
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};       // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                               // make PG0 in (HiZ)
    GPIO_PORTG_AFSEL_R &= ~0x01;                            // disable alt funct on PG0
    GPIO_PORTG_DEN_R |= 0x01;                               // enable digital I/O on PG0
  
    GPIO_PORTG_AMSEL_R &= ~0x01;                             // disable analog functionality on PG0

    return;
}
void PortH_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 	// Activate the clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};					// Allow time for clock to stabilize
	
		
	GPIO_PORTH_DIR_R=0b00001111;															// Enable PH0 - PH3 as outputs
	GPIO_PORTH_DEN_R=0b00001111;															// Enable PH0 - PH3 as digital pins
	return;
}
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 	// Activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};					// Allow time for clock to stabilize
	
		
	GPIO_PORTF_DIR_R=0b00010001;															// Enable PF0 & PF4 as output
	GPIO_PORTF_DEN_R=0b00010001;															// Enable PF0 & PF4 as digital pin
	return;
}
//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                       // make PG0 input (HiZ)
    
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
	uint16_t wordData;
	uint16_t Distance;
	uint16_t SignalRate;
	uint16_t AmbientRate;
	uint16_t SpadNum; 
	uint8_t RangeStatus;
	uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	PortM_Init();
	PortF_Init();
	GPIO_PORTF_DATA_R = 0b00000000;
	GPIO_PORTM_DATA_R = 0b00000000;

	// 1 Wait for device ToF booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  	}

	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
	/* 2 Initialize the sensor with the default setting  */
	status = VL53L1X_SensorInit(dev);

	/* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
	status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
	//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
	//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

	status = VL53L1X_StartRanging(dev);   // 4 This function has to be called to enable the ranging
	
	//initialize variables
	int steps=0;
	double angle = 0;
	float x;
	float y;
	float z = 0;
	double pi = 3.1415;
	int count = 0;
	int input = 0;
	int p;
	int spin=0;
	
	while(1){
		input = UART_InChar();  //press enter to start
		if (input == 's')
			break;
	}
	
	SysTick_Wait10ms(10);

	for(p=0;p<3;p++){
		while(1){
		if((GPIO_PORTM_DATA_R & 0b00000010)==0)          //start each scan by pressing the button
			break;
	}	

	while(steps < 2048){
		GPIO_PORTH_DATA_R = 0b00001100;   //turn stepper motor
		SysTick_Wait(160000);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait(160000);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait(160000);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait(160000);
		steps += 4;

		if (steps % 124 == 0){
		    GPIO_PORTF_DATA_R = 0b00010000;
			SysTick_Wait(1600000);
			GPIO_PORTF_DATA_R = 0b00000000;
			
			//5 wait until the ToF sensor's data is ready
			while (dataReady == 0){
		    status = VL53L1X_CheckForDataReady(dev, &dataReady);
            VL53L1_WaitMs(dev, 5);
	    }
		
		dataReady = 0;
	    angle += 22.5;

		//7 read the data values from ToF sensor
		/*
		status = VL53L1X_GetRangeStatus(dev, &RangeStatus);           //Uncomment to print range status
		sprintf(printf_buffer,"Range Status: %u\r\n", RangeStats);
		UART_printf(printf_buffer);
		*/
	    status = VL53L1X_GetDistance(dev, &Distance);					//7 The Measured Distance value
		x = Distance*sin(angle*(pi/180));
		y = Distance*cos(angle*(pi/180));
	    status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/
		
		// print the resulted readings to UART
		sprintf(printf_buffer,"%f, %f, %f\r\n", x, y,z);
		UART_printf(printf_buffer);
	    SysTick_Wait10ms(50);
        }
	}
	    
	GPIO_PORTF_DATA_R = 0b00000001;
	SysTick_Wait(16000000);
	GPIO_PORTF_DATA_R = 0b00000000;
	steps=0;
		
	while(steps < 2048){                //return stepper motor to home position (to avoid wires tangling)
		GPIO_PORTH_DATA_R = 0b00001001;
      	SysTick_Wait(160000);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait(160000);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait(160000);
		GPIO_PORTH_DATA_R = 0b000011000;
		SysTick_Wait(160000);
		steps += 4;}
		steps=0;
		z+=10; //increase the z value by 10 (1cm)
		}
	}
