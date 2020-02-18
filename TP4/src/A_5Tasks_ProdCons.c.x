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
#include "semphr.h"

/* Priorities at which the tasks are created. */
#define CONSUMER_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define PRODUCER_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define ERROR_TASK_PRIORITY (tskIDLE_PRIORITY)
#define REPRIORITIZE_TASK_PRIORITY (tskIDLE_PRIORITY + configMAX_PRIORITIES)

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

static void prvConsumerATask(void *pvParameters);
static void prvConsumerBTask(void *pvParameters);
static void prvConsumerCTask(void *pvParameters);
static void prvProducerATask(void *pvParameters);
static void prvProducerBTask(void *pvParameters);
static void prvErrorTask(void *pvParameters);
static void vReprioritizeTask(void *pvParameters);
void vConfigureTimerForRunTimeStats(void);
static void prvToggleLED(void);
int arrayIsUnique(UBaseType_t *);

/* Declare a variable of type QueueHandle_t. This is used to store the handle
to the queue that is accessed by all two tasks. */
static QueueHandle_t xQueue = NULL;
/* Create a mutex type semaphore. */
SemaphoreHandle_t xSemaphore = NULL;
/* Manejadores de las tareas. */
TaskHandle_t xHandleConsumerA;
TaskHandle_t xHandleConsumerB;
TaskHandle_t xHandleConsumerC;
TaskHandle_t xHandleProducerA;
TaskHandle_t xHandleProducerB;

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
	/* The mutexes are the better choice for implementing simple mutual exclusion. */
	xSemaphore = xSemaphoreCreateMutex();

	if ((xQueue != NULL) && (xSemaphore != NULL))
	{
		xTaskCreate(prvConsumerATask, "ConsumidorA", OWN_STACK_SIZE, NULL, CONSUMER_TASK_PRIORITY, xHandleConsumerA);
		xTaskCreate(prvConsumerBTask, "ConsumidorB", OWN_STACK_SIZE, NULL, CONSUMER_TASK_PRIORITY, xHandleConsumerB);
		xTaskCreate(prvConsumerCTask, "ConsumidorC", OWN_STACK_SIZE, NULL, CONSUMER_TASK_PRIORITY, xHandleConsumerC);
		xTaskCreate(prvProducerATask, "ProductorA", OWN_STACK_SIZE, NULL, PRODUCER_TASK_PRIORITY, xHandleProducerA);
		xTaskCreate(prvProducerBTask, "ProductorB", OWN_STACK_SIZE, NULL, PRODUCER_TASK_PRIORITY, xHandleProducerB);
		xTaskCreate(vReprioritizeTask, "Reprioritize", configMINIMAL_STACK_SIZE, NULL, REPRIORITIZE_TASK_PRIORITY, NULL);
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
 * @brief Tarea que se ejecuta cada 5 segundos alterando las prioridades
 *        de las demás tareas. El valor de prioridad asignado y aleatorio
 * 		  pero siempre mayor que la prioridad tskIDLE_PRIORITY y menor a
 *   	  configMAX_PRIORITIES (FreeRTOSConfig.h).
 * 
 * @param pvParameters 
 */
static void vReprioritizeTask(void *pvParameters)
{
	UBaseType_t xNewPriority[5];
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	traceString cht = xTraceRegisterString("Reprioritize");
	const TickType_t xPeriod = pdMS_TO_TICKS(3000);

	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		vTracePrint(cht, "T_Reprioritize");
		do
		{

			for (int i = 0; i < 5; i++)
			{
				xNewPriority[i] = ((rand() % (configMAX_PRIORITIES - 3)) + 1);
			}
		} while (arrayIsUnique(xNewPriority));

		vTaskPrioritySet(xHandleConsumerA, xNewPriority[0]);
		vTaskPrioritySet(xHandleConsumerB, xNewPriority[1]);
		vTaskPrioritySet(xHandleConsumerC, xNewPriority[2]);
		vTaskPrioritySet(xHandleProducerA, xNewPriority[3]);
		vTaskPrioritySet(xHandleProducerB, xNewPriority[4]);
	}
}

int arrayIsUnique(UBaseType_t *xNewPriority)
{
	uint8_t xIsUnique = 0;
	for (uint8_t i = 0; i < 5; i++)
	{
		for (uint8_t j = 0; j < 5; j++)
		{

			//Checking if the indexes are different if they are then we will compare the element at position i with the element at position j
			if (i != j)
			{
				// Comparing elements in the array with other elements in the array if these are equal then the array of characters are not unique
				if (xNewPriority[i] == xNewPriority[j])
				{
					xIsUnique = 1; // Updating isUnique variable to be equal to 0 to represent false
				}
			}
		}
	}
	return xIsUnique;
}

/* ======================================================================================== */
/**
 * @brief Agrega un elemento aleatorio a la cola compartida por las tareas.
 * Espera un tiempo mainQUEUE_SEND_FREQUENCY_MS para activarse.
 * 
 * @param pvParameters 
 */
static void prvProducerATask(void *pvParameters)
{
	portTickType xNextWakeTime;
	uint8_t xRandomValue;
	BaseType_t xStatus;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();
	traceString cht = xTraceRegisterString("CanalProductorA");

	for (;;)
	{
		vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);
		/* Anuncio en Tracealyzer */
		vTracePrint(cht, "T_ProductorA");
		xRandomValue = (rand() % 254);
		if (xSemaphore != NULL)
		{
			/* Si el semaforo esta libre, lo tomo. */
			if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(2)) == pdTRUE)
			{
				/* Verificamos si hay espacio disponible en la cola */
				if (uxQueueSpacesAvailable(xQueue) > 0)
				{
					/* Comprobamos estado de la operacion */
					xStatus = xQueueSend(xQueue, &xRandomValue, 0);
					/*if (xStatus != pdPASS)
						vPrintString("No se puedo enviar elemento a la cola.\r\n");
					else
					{
						vPrintString(pcTaskGetName(xHandleProducerA));
						vPrintString(" - Elemento enviado: ");
						vPrintStringAndNumber("", xRandomValue);
						vPrintString("\r\n");
					}*/
				}
				/* Libero el semaforo. */
				xSemaphoreGive(xSemaphore);
			}
		}
	}
}

static void prvProducerBTask(void *pvParameters)
{
	portTickType xNextWakeTime;
	uint8_t xRandomValue;
	BaseType_t xStatus;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();
	traceString cht = xTraceRegisterString("CanalProductorA");

	for (;;)
	{
		vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);
		/* Anuncio en Tracealyzer */
		vTracePrint(cht, "T_ProductorB");
		xRandomValue = (rand() % 254);
		if (xSemaphore != NULL)
		{
			/* Si el semaforo esta libre, lo tomo. */
			if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(2)) == pdTRUE)
			{
				/* Verificamos si hay espacio disponible en la cola */
				if (uxQueueSpacesAvailable(xQueue) > 0)
				{
					/* Comprobamos estado de la operacion */
					xStatus = xQueueSend(xQueue, &xRandomValue, 0);
					/*if (xStatus != pdPASS)
						vPrintString("No se puedo enviar elemento a la cola.\r\n");
					else
					{
						vPrintString(pcTaskGetName(xHandleProducerB));
						vPrintString(" - Elemento enviado: ");
						vPrintStringAndNumber("", xRandomValue);
						vPrintString("\r\n");
					}*/
				}
				/* Libero el semaforo. */
				xSemaphoreGive(xSemaphore);
			}
		}
	}
}

/* ======================================================================================== */
/* ======================================================================================== */

/**
 * @brief Quita un elemento de la cola compartida entre las tareas.
 * 
 * @param pvParameters 
 */
static void prvConsumerATask(void *pvParameters)
{
	portTickType xNextWakeTime;
	uint8_t xRandomReceive;
	BaseType_t xStatus;
	traceString chr = xTraceRegisterString("CanalConsumidorA");

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for (;;)
	{
		vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);
		/* Anuncio en Tracealyzer */
		vTracePrint(chr, "T_ConsumidorA");
		if (xSemaphore != NULL)
		{
			/* Si el semaforo esta libre, lo tomo. */
			if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(2)) == pdTRUE)
			{
				xStatus = xQueueReceive(xQueue, &xRandomReceive, 0);
				/*if (xStatus == pdPASS)
				{
					vPrintString(pcTaskGetName(xHandleConsumerA));
					vPrintString(" - Elemento quitado: ");
					vPrintStringAndNumber("", xRandomReceive);
					vPrintString("\r\n");
				}
				else
				{
					vPrintString("No se pudo tomar elemento de la cola.");
					vPrintString("\r\n");
				}*/
				/* Libero el semaforo. */
				xSemaphoreGive(xSemaphore);
			}
		}
	}
}

static void prvConsumerBTask(void *pvParameters)
{
	portTickType xNextWakeTime;
	uint8_t xRandomReceive;
	BaseType_t xStatus;
	traceString chr = xTraceRegisterString("CanalConsumidorB");

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for (;;)
	{
		vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);
		/* Anuncio en Tracealyzer */
		vTracePrint(chr, "T_ConsumidorB");
		if (xSemaphore != NULL)
		{
			/* Si el semaforo esta libre, lo tomo. */
			if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(2)) == pdTRUE)
			{
				xStatus = xQueueReceive(xQueue, &xRandomReceive, 0);
				/*if (xStatus == pdPASS)
				{
					vPrintString(pcTaskGetName(xHandleConsumerB));
					vPrintString(" - Elemento quitado: ");
					vPrintStringAndNumber("", xRandomReceive);
					vPrintString("\r\n");
				}
				else
				{
					vPrintString("No se pudo tomar elemento de la cola.");
					vPrintString("\r\n");
				}*/
				/* Libero el semaforo. */
				xSemaphoreGive(xSemaphore);
			}
		}
	}
}

static void prvConsumerCTask(void *pvParameters)
{
	portTickType xNextWakeTime;
	uint8_t xRandomReceive;
	BaseType_t xStatus;
	traceString chr = xTraceRegisterString("CanalConsumidorC");

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for (;;)
	{
		vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);
		/* Anuncio en Tracealyzer */
		vTracePrint(chr, "T_ConsumidorC");
		if (xSemaphore != NULL)
		{
			/* Si el semaforo esta libre, lo tomo. */
			if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(2)) == pdTRUE)
			{
				xStatus = xQueueReceive(xQueue, &xRandomReceive, 0);
				/*if (xStatus == pdPASS)
				{
					vPrintString(pcTaskGetName(xHandleConsumerC));
					vPrintString(" - Elemento quitado: ");
					vPrintStringAndNumber("", xRandomReceive);
					vPrintString("\r\n");
				}
				else
				{
					vPrintString("No se pudo tomar elemento de la cola.");
					vPrintString("\r\n");
				}*/
				/* Libero el semaforo. */
				xSemaphoreGive(xSemaphore);
			}
		}
	}
}
/* ======================================================================================== */

/**
 * @brief Conmuta el estado del led entre encendido/apagado.
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
