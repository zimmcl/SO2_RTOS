/**
 * @file A_2Tasks_ProdCons.c
 * @author Ezequiel Zimmel (ezequielzimmel@gmail.com)
 * @brief Proyecto clásico de Producto/Consumidor con cola compartida
 * entre las tareas. Se asignan distintos niveles de prioridad.
 * @version 0.1
 * @date 2020-02-03
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "basic_io.h"

/* Priorities at which the tasks are created. */
#define RECEIVE_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define SEND_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define ERROR_TASK_PRIORITY (tskIDLE_PRIORITY)

/* The bit of port 0 that the LPCXpresso LPC13xx LED is connected. */
#define mainLED_BIT (22)

/* The rate at which data is sent to the queue, specified in milliseconds. */
#define mainQUEUE_SEND_FREQUENCY_MS (10 / portTICK_RATE_MS)

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH (100)

/*  */
#define OWN_STACK_SIZE 1000

static void prvConsumerTask(void *pvParameters);
static void prvProducerTask(void *pvParameters);
static void prvErrorTask(void *pvParameters);
void vConfigureTimerForRunTimeStats(void);
static void prvToggleLED(void);

/* Declare a variable of type QueueHandle_t. This is used to store the handle
to the queue that is accessed by all two tasks. */
static QueueHandle_t xQueue = NULL;

/**
 * @brief Se configura el modo de funcionamiento del PIN 0.22
 * Se inicializa Trace y se reserva memoria para la cola compartida.
 * Se crean las tareas y se ejecuta el planificador.
 * 
 * @return int 
 */
int main(void)
{
	/* Initialise P0_22 for the LED. */
	LPC_PINCON->PINSEL1 &= (~(3 << 12));
	LPC_GPIO0->FIODIR |= (1 << mainLED_BIT);

	/* Init and start tracing */
	vTraceEnable(TRC_START);

	/* Create the queue. */
	xQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint8_t));

	if (xQueue != NULL)
	{
		/* Start the two tasks. */
		xTaskCreate(prvConsumerTask, "Consumidor", configMINIMAL_STACK_SIZE, NULL, RECEIVE_TASK_PRIORITY, NULL);
		xTaskCreate(prvProducerTask, "Productor", configMINIMAL_STACK_SIZE, NULL, SEND_TASK_PRIORITY, NULL);
		/* Start the tasks running. */
		vTaskStartScheduler();
	}
	else
	{
		xTaskCreate(prvErrorTask, "ERROR", configMINIMAL_STACK_SIZE, NULL, ERROR_TASK_PRIORITY, NULL);
		vTaskStartScheduler();
	}

	for (;;)
		;
}

/**
 * @brief Tarea que se ejecuta de manera indefinida en caso de generarse
 * un problema en la reserva de memoria en el Heap.
 * Realiza la conmutación entre encendido/apagado del Led integrado en la 
 * placa.
 * 
 * @param pvParameters 
 */
static void prvErrorTask(void *pvParameters)
{
	for (;;)
	{
		prvToggleLED();
		vTaskDelay(750);
	}
}

/**
 * @brief Agrega un elemento aleatorio a la cola compartida por las tareas.
 * Espera un tiempo mainQUEUE_SEND_FREQUENCY_MS para activarse.
 * 
 * @param pvParameters 
 */
static void prvProducerTask(void *pvParameters)
{
	portTickType xNextWakeTime;
	uint8_t xRandomValue;
	BaseType_t xStatus;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();
	traceString cht = xTraceRegisterString("CanalProductor");

	for (;;)
	{
		/* Genero el elemento aleatorio para insertar en la cola. */
		xRandomValue = (rand() % 254);
		/* Verificamos si hay espacio disponible en la cola */
		if (uxQueueSpacesAvailable(xQueue) > 0)
		{
			/* Anuncio en Tracealyzer */
			vTracePrint(cht, "T_Productor");
			vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);
			/* Comprobamos estado de la operacion */
			xStatus = xQueueSend(xQueue, &xRandomValue, 0);
			if (xStatus != pdPASS)
				vPrintString("No se puedo enviar elemento a la cola.\r\n");
			else
			{
				vPrintString("Elemento enviado a la cola - ");
				vPrintStringAndNumber("Valor: ", xRandomValue);
				vPrintString("\r\n");
			}
		}
	}
}

/**
 * @brief Quita un elemento de la cola compartida entre las tareas.
 * 
 * @param pvParameters 
 */
static void prvConsumerTask(void *pvParameters)
{
	uint8_t xRandomReceive;
	BaseType_t xStatus;
	traceString chr = xTraceRegisterString("CanalConsumidor");

	for (;;)
	{
		xStatus = xQueueReceive(xQueue, &xRandomReceive, portMAX_DELAY);
		if (xStatus == pdPASS)
		{
			/* Anuncio en Tracealyzer */
			vTracePrint(chr, "T_Consumidor");
			vPrintString("Elemento quitado de la cola - ");
			vPrintStringAndNumber("Valor: ", xRandomReceive);
			vPrintString("\r\n");
		}
		else
		{
			vPrintString("No se pudo tomar elemento de la cola.\r\n");
		}
	}
}

/**
 * @brief Conmuta el estado del led integrado a la placa, pin P0.22, entre
 *        encendido/apagado.
 * 
 */
static void prvToggleLED(void)
{
	unsigned long ulLEDState;
	/* Obtain the current P0 state. */
	ulLEDState = LPC_GPIO0->FIOPIN;
	/* Turn the LED off if it was on, and on if it was off. */
	LPC_GPIO0->FIOCLR = ulLEDState & (1 << mainLED_BIT);
	LPC_GPIO0->FIOSET = ((~ulLEDState) & (1 << mainLED_BIT));
}

void vConfigureTimerForRunTimeStats(void)
{
	const unsigned long TCR_COUNT_RESET = 2, CTCR_CTM_TIMER = 0x00, TCR_COUNT_ENABLE = 0x01;
	/* Power up and feed the timer. */
	LPC_SC->PCONP |= 0x02UL;
	LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & (~(0x3 << 2))) | (0x01 << 2);
	/* Reset Timer 0 */
	LPC_TIM0->TCR = TCR_COUNT_RESET;
	/* Just count up. */
	LPC_TIM0->CTCR = CTCR_CTM_TIMER;
	/* Prescale to a frequency that is good enough to get a decent resolution,
	but not too fast so as to overflow all the time. */
	LPC_TIM0->PR = (configCPU_CLOCK_HZ / 10000UL) - 1UL;
	/* Start the counter. */
	LPC_TIM0->TCR = TCR_COUNT_ENABLE;
}
