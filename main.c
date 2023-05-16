/*  Including Necessary Libs  */

//FREERTOS LIBS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "tm4c123gh6pm.h"
//Tivaware Libs
#include "FreeRTOSConfig.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
//Standard Libs
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Declaring SemaphoreHandle_t objects, which are used in FreeRTOS to manage and synchronize tasks.
	 These semaphores provide synchronization and mutual exclusion between tasks.
   Tasks can use these semaphores to coordinate and communicate with each other effectively.*/ 
SemaphoreHandle_t WindowUp_driverS;
SemaphoreHandle_t WindowDown_driverS;
SemaphoreHandle_t WindowUp_passengerS;
SemaphoreHandle_t WindowDown_passengerS;
SemaphoreHandle_t ObstacleS;
SemaphoreHandle_t OnOffS;

/* By acquiring the semaphore before accessing the motor.
   A Task can ensure exclusive access to the motor and prevent concurrent access from other tasks*/
SemaphoreHandle_t MotorMutex;			

/* Creating a handle to a Queue */
xQueueHandle xQueue;

/*  Tasks Functions Prototypes  */
void initTask            (void *params);
void WindowUp_Driver     (void *params);
void WindowDown_Driver   (void *params);
void OnOff               (void *params);
void WindowUp_Passenger  (void *params);
void WindowDown_Passenger(void *params);
void Obstacle            ();

/*  Function Prototypes For Interrupt Handlers  
    Interrupt Handlers are called when an Interrupt event occurs on the associated GPIO port. */
void GPIOA_Handler();
void GPIOF_Handler();
void GPIOC_Handler();

/* Function Takes Unsigned 32-bit Integer , 'n' Milliseconds */
void delayMs(uint32_t n)
{
  int i,j;               //Local Variables
  for(i=0;i<n;i++)       //Nested for loop for Total delay
  {
    for(j=0;j<3180;j++)  //Empty loop to delay '1' milliseconds, 3180 Iterations (achieve desired time)

    {}
  }
}


int main()
{
	/*  Creating Semaphores  */
	WindowUp_driverS      = xSemaphoreCreateBinary();
	WindowDown_driverS    = xSemaphoreCreateBinary();
	OnOffS                = xSemaphoreCreateBinary();
	WindowUp_passengerS   = xSemaphoreCreateBinary();
	WindowDown_passengerS = xSemaphoreCreateBinary();
	ObstacleS             = xSemaphoreCreateBinary();
	
	MotorMutex            = xSemaphoreCreateMutex();
	
	/*  Creating Tasks  */
	xTaskCreate(initTask,             "init",                 80, NULL, 4, NULL);
	xTaskCreate(WindowUp_Driver,      "WindowUp_Driver",      80, NULL, 2, NULL);
	xTaskCreate(WindowDown_Driver,    "WindowDown_Driver",    80, NULL, 2, NULL);
	xTaskCreate(WindowUp_Passenger,   "WindowUp_Passenger",   80, NULL, 1, NULL);
	xTaskCreate(WindowDown_Passenger, "WindowDown_Passenger", 80, NULL, 1, NULL);
	xTaskCreate(OnOff,                "OnOff",                80, NULL, 2, NULL);
	
	/*  Creating Queue  */
	xQueue = xQueueCreate( 1, sizeof(int));

	vTaskStartScheduler();   // Start the FreeRTOS scheduler
	/* vTaskStartScheduler() function typically marks the transition from the setup/configuration phase to the actual execution phase of a FreeRTOS application.
     After calling this function, the scheduler takes control and begins executing the tasks as per their scheduling parameters. */

	while (1){}
}

void initTask(void *params)
{
	for (;;)
	{

		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);         //Enabling Clock to PORT A
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)); //Loop Until PortA is Enabled
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);         //Enabling Clock to PORT B
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)); //Loop Until PortB is Enabled
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);         //Enabling Clock to PORT C
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)); //Loop Until PortC is Enabled
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);         //Enabling Clock to PORT E
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)); //Loop Until PortE is Enabled
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);				 //Enabling Clock to PORT F
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); //Loop Until PortF is Enabled

		/*Configuring The Following Pins to act as an Inputs Pins :
		1) PORTA Pins 2, 3, 6 and 7
		2) PORTB Pins 6 and 7
		3) PORTC Pin 5
		4) PORTE Pins 1 and 2
		5) PORTF Pin 4*/
		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_6 | GPIO_PIN_7); // UP & DOWN --> Driver and Passenger
		GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 );                         // Window Limit Switches
		GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5);                                       // Jamming
		GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);                                       // Child Lock
		
		//Configuring as Pull-up (Zero Check)
		GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /*Configuring PORTE Pins 1 and 2 to act as an Outputs Pins*/
		GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);                         //Motor Pins          
		GPIOUnlockPin(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);
		
		/*Enabling Interrupts for The following Pins
			1)PORTA Pins 2, 3, 6 and 7 
			2)PORTC Pin 5
			3)PortF Pin 4*/	
		GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_6 | GPIO_PIN_7);
		GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_5);
		GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4 );
		
		/*Setting Interrupt Triggering Type as Falling Edge
			1)PORTA Pins 2, 3, 6 and 7 
			2)PORTC Pin 5
			3)PortF Pin 4*/	
		GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3, GPIO_FALLING_EDGE);
		GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
		GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
		
		//Registering Interrupt Handler Functions for Ports A, C and F
		GPIOIntRegister(GPIO_PORTA_BASE, GPIOA_Handler);
		GPIOIntRegister(GPIO_PORTC_BASE, GPIOC_Handler);
		GPIOIntRegister(GPIO_PORTF_BASE, GPIOF_Handler);
		
		//Setting The Interrupt Piorities
		IntPrioritySet(INT_GPIOA, 0XE0);	//Priority Seven
		IntPrioritySet(INT_GPIOC, 0XE0);			
		IntPrioritySet(INT_GPIOF, 0XE0);
	

		
		//Task will run once then it will suspend itself
		vTaskSuspend(NULL); 
	}
}

void GPIOA_Handler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2))
	{
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);
		xSemaphoreGiveFromISR(WindowUp_driverS, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}

	else if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3))
	{
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);
		xSemaphoreGiveFromISR(WindowDown_driverS, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		
	}
	else if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6))
	{
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
		xSemaphoreGiveFromISR(WindowUp_passengerS, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	else if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))
	{
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);
		xSemaphoreGiveFromISR(WindowDown_passengerS, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	else{
			GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);
			GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);
			GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);
			GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
	}
}

void GPIOC_Handler(){
			GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_5);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			int flag=1;
			xQueueSendFromISR(xQueue,&flag,&xHigherPriorityTaskWoken);
}

void GPIOF_Handler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if (!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4))
	{
			GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
		xSemaphoreGiveFromISR(OnOffS, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}

	else{
		GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);

	}
}


void WindowUp_Driver(void *params)
{
	int out = 0;
	xSemaphoreTake(WindowUp_driverS, 0);
	while (1)
	{
		xSemaphoreTake(WindowUp_driverS, portMAX_DELAY); //Taking WindowUp_driverS in order to proceed
		xSemaphoreTake(MotorMutex, portMAX_DELAY); //Taking MotorMutex as it requires an exclusive access to a the DC Motor which is shared resource
	
	// jamming is not pressed
	if ((GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)))
	{
		 //Checking if up 
			if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2))
		{
			delayMs(50); // For Debouncing 
		
		//Start DC Motor
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x0);
		delayMs(1000);  //wait for a second to check whether manual or automatic 
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2)) // if not pressed
		{
			//automatic		
			while(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6))
			{
				if (xQueueReceive( xQueue, &out, 0 )== pdTRUE) //detecting obstacle flag
				{
						Obstacle();
					break;
				}
				if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) || !GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)) //either down buttons are pressed
				{
					break;
				}
			}
		}
			else
				{
					
					while (!(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2)) && GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6)) //still pressed & no limit switch
					{
						if (xQueueReceive( xQueue, &out, 0 )== pdTRUE) //detecting obstacle flag
				{
						Obstacle();
					break;
				}
						
					}
						GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x00); //stop motor
			}
		}
	}
	else
	{
		__WFI();
	}

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x00); //stop motor
		xSemaphoreGive(MotorMutex);
	}

}


void WindowDown_Driver(void *params)
{
	xSemaphoreTake(WindowDown_driverS, 0);
	while (1)
	{
		xSemaphoreTake(WindowDown_driverS, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
			if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3))
		{

			delayMs(50);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x0);
		delayMs(1000);
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3))
		{
			//automatic
			while(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7))
			{
				if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) || !GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6))
				{
					break;
				}
				
			}
		}
			else
				//manual
				{			
					while (!(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3)) && GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7))
					{
						
					}
						GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x00);
			}
	}		
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x00);
		xSemaphoreGive(MotorMutex);
	
}
	}

void WindowUp_Passenger(void *params)
{
	int out=0;
	xSemaphoreTake(WindowUp_passengerS, 0);
	while (1)
	{
		xSemaphoreTake(WindowUp_passengerS, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		
	if ((GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)))
	{
		if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6))
		{
			delayMs(50);
		
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x00);
		delayMs(1000);
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6))
		{
			
			//automatic
			while(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6))
			{
				if (xQueueReceive( xQueue, &out, 0 )== pdTRUE) //detecting obstacle flag
				{
						Obstacle();
					break;
				}
				if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) || !GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))
				{
					break;
				}
				}
		
			}
		
			else 
				{
					while (!(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6)) && GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6) )
					{
						if (xQueueReceive( xQueue, &out, 0 )== pdTRUE) //detecting obstacle flag
				{
						Obstacle();
					break;
				}
						
					}
						GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x00);
			}
		}
	}
	else
		{
			__WFI();
	}
		
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x00);
		xSemaphoreGive(MotorMutex);
	}
	}

void WindowDown_Passenger(void *params)
{
	xSemaphoreTake(WindowDown_passengerS, 0);
	while (1)
	{
		xSemaphoreTake(WindowDown_passengerS, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		
		if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))
		{

			delayMs(50);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x0);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
		
		delayMs(1000);
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))
		{
			//automatic
			
			while(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7))
			{
				if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) || !GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6))
				{
					break;
				}
			}
		}
			else
				{				
					//manual
					while (!(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)) && GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7))
					{
						
						
					}
						GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x00);
			}
		}
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x00);
		xSemaphoreGive(MotorMutex);
	}
}

void OnOff(void *params)
{
	xSemaphoreTake(OnOffS, 0);
	while (1)
	{
		xSemaphoreTake(OnOffS, portMAX_DELAY);	
			
		while (!(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)))
		{
			GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7 );
		__WFI();
		}
			GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7 );
	}
}

void Obstacle ()
{
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x00);
	delayMs(200);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x0);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
		delayMs(500);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2,0x00);
	
}