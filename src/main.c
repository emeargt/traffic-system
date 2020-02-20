/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100

enum light
{
    RED = 0b001,
	YELLOW = 0b010,
	GREEN = 0b100
};

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/* Initialization Functions */
static void spi_init( void );
static void gpioA_init( void );
static void gpioB_init( void );
static void adc_init( void );

/* Other Functions */
void mySPI_send(SPI_TypeDef *SPIx, uint8_t data);

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void Flow_Adjust_Task( void *pvParameters );
static void Traffic_Gen_Task( void *pvParameters );
static void Light_State_Task( void *pvParameters );
static void Sys_Display_Task( void *pvParameters );

xQueueHandle xQ_flow_rate = 0;
xQueueHandle xQ_light_period = 0;
xQueueHandle xQ_light_state = 0;
xQueueHandle xQ_car_gen = 0;

/*-----------------------------------------------------------*/

int main(void)
{
	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();
	adc_init();
	gpioA_init();
	gpioB_init();
	spi_init();

	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	xQ_flow_rate = xQueueCreate( mainQUEUE_LENGTH, sizeof( double ) );
	xQ_light_period = xQueueCreate( 1, sizeof( double ) );
	xQ_light_state = xQueueCreate( 1, sizeof( uint8_t ) );
	xQ_car_gen = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint8_t ) );

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xQ_flow_rate, "FlowRateQueue" );
	vQueueAddToRegistry( xQ_light_period, "LightPeriodQueue" );
	vQueueAddToRegistry( xQ_light_state, "LightStateQueue" );
	vQueueAddToRegistry( xQ_car_gen, "CarGenQueue" );

	xTaskCreate( Flow_Adjust_Task, "Flow-Adjust", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( Traffic_Gen_Task, "Traffic-Gen", configMINIMAL_STACK_SIZE, NULL, 0, NULL );
	xTaskCreate( Light_State_Task, "Light-State", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
	xTaskCreate( Sys_Display_Task, "Sys-Display", configMINIMAL_STACK_SIZE, NULL, 0, NULL );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}


/*-----------------------------------------------------------*/

static void Flow_Adjust_Task( void *pvParameters )
{
	while(1)
	{
		double adc_norm = (double)ADC_GetConversionValue(ADC1) / 4095;
		double flow_rate = (0.8 * adc_norm + 0.2) * 100; // car gen probability between 20 - 100
		xQueueOverwrite( xQ_light_period, &adc_norm );
		if( xQueueSend( xQ_flow_rate, &flow_rate, 1000 ) )
		{
			vTaskDelay(1000);
		}
		else
		{
			printf("Flow Adjust Task: Failed to add flow rate to queue.\n");
		}
	}
}

static void Traffic_Gen_Task( void *pvParameters )
{
	while(1)
	{
		double prob = 0.0;
		if( xQueueReceive( xQ_flow_rate, &prob, 1000 ) )
		{
			uint8_t car_val = (rand() % 100) < prob;
			if( !xQueueSend( xQ_car_gen, &car_val, 1000 ) )
			{
				printf("Traffic Gen Task: Failed to add new car (or non-car) to the queue.\n");
			}
		}
		else
		{
			printf("Traffic Gen Task: No flow rate values available on the queue.\n");
		}
		vTaskDelay(1000);
	}
}

static void Light_State_Task( void *pvParameters )
{
	enum light state = GREEN;
	unsigned int durations[3] = {0, 2, 0}; // red, yellow green
	while(1)
	{
		double flow = 0.0;
		if( xQueuePeek( xQ_light_period, &flow, 0 ) )
		{
			durations[2] = (unsigned int)(7 * flow + 3); // update green light period
			durations[0] = (unsigned int)(-7 * flow + 10); // update red light period
		}
		else
		{
			printf("Light State Task: No flow rate values to peek at.\n");
		}
		switch(state)
		{
		case GREEN:
			xQueueOverwrite( xQ_light_state, &state );
			state = YELLOW;
			vTaskDelay(durations[2] * 1000); // wait till end of green light
			break;
		case YELLOW:
			xQueueOverwrite( xQ_light_state, &state );
			state = RED;
			vTaskDelay(durations[1] * 1000); // wait till end of yellow light
			break;
		case RED:
			xQueueOverwrite( xQ_light_state, &state );
			state = GREEN;
			vTaskDelay(durations[0] * 1000); // wait till end of red light
			break;
		}

	}
}

static void Sys_Display_Task( void *pvParameters )
{
	uint8_t traffic_lower = 0x00;
	uint16_t traffic_upper = 0x0000;
	uint8_t light_state = 0x00;
	while(1)
	{
		uint8_t car = 0;
		xQueuePeek( xQ_light_state, &light_state, 0 );
		if( xQueueReceive( xQ_car_gen, &car, 0 ) )
		{
			if(light_state != GREEN)
			{
				traffic_upper = traffic_upper << 1;
				traffic_lower |= (traffic_lower << 1) | (car & 0x1);
			}
			else
			{
				traffic_upper = (traffic_upper << 1) | ((traffic_lower & 0x80) >> 7);
				traffic_lower = (traffic_lower << 1) | (car & 0x1);
			}
		}
		uint8_t send1 = ((light_state << 5) & 0xE0) | ((traffic_upper & 0x0700) >> 8);
		uint8_t send2 = traffic_upper & 0x00FF;
		mySPI_send(SPI1, send1);
		mySPI_send(SPI1, send2);
		mySPI_send(SPI1, traffic_lower);

		vTaskDelay(500);
	}
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

static void spi_init( void )
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); // enable periph clk for SPI1

	// Init SPI interface
	SPI_InitTypeDef SPI1_InitStruct;
	SPI1_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI1_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI1_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI1_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI1_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI1_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI1_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI1_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI1_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI1_InitStruct);
	SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
	SPI_CalculateCRC(SPI1, DISABLE);
	SPI_Cmd(SPI1, ENABLE);
}

static void gpioA_init( void )
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // enable clocks for SPI1 pins on GPIOA

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); // SPI1_SCK
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); // SPI1_MISO
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); // SPI1_MOSI

	GPIO_InitTypeDef GPIOA_InitStruct;
	GPIOA_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIOA_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIOA_InitStruct);
}

static void gpioB_init( void )
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1); // SPI1_SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1); // SPI1_MOSI

	GPIO_InitTypeDef GPIOB_InitStruct;
	GPIOB_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
	GPIOB_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOB_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOB_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOB, &GPIOB_InitStruct);
}

// Initialize and enable the ADC peripheral
static void adc_init( void )
{
	// Enable Periph Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	// Single ADC mode
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	// ADCCLK = PCLK2/2
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	// Available only for multi ADC mode
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	// Delay between 2 sampling phases
	ADC_CommonInitStruct.ADC_TwoSamplingDelay =	ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);

	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);

	// Write ADC Configuration to register
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1,ENABLE);
	ADC_ContinuousModeCmd(ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);
}

void mySPI_send(SPI_TypeDef *SPIx, uint8_t data)
{
	while((SPIx->SR & SPI_SR_BSY) != 0x0000);
	SPIx->DR = (uint16_t)data;
	while((SPIx->SR & SPI_SR_BSY) != 0x0000);
}
