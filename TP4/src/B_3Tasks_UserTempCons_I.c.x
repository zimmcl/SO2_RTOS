/**
 * @file B_3Tasks_UserTempCons_R.c
 * @author Ezequiel Zimmel (ezequielzimmel@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-02-03
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <math.h>
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Priorities at which the tasks are created. */
#define RECEIVE_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define PERIODIC_TASK_PRIORITY (tskIDLE_PRIORITY + 4)
#define APERIODIC_TASK_PRIORITY (tskIDLE_PRIORITY + 4)
/* The bit of port 0 that the LPCXpresso LPC13xx LED is connected. */
#define mainLED_BIT (22)
#define MAX_POINTERS 10

/* The rate at which data is sent to the queue, specified in milliseconds. */
#define PERIODIC_SEND_FREQUENCY_MS (50 / portTICK_RATE_MS)

//Defines del programa
#define MAX_STRING 60
#define TMP_INT 5
#define portNEW_DELAY 200

/* Interrupcion externa EINT0 */
#define PINSEL_EINT0 20
#define SBIT_EINT0 0
#define SBIT_EXTMODE0 0
#define SBIT_EXTPOLAR0 1
#define ULONG_MAX 0xFFFFFFFF

#define PRIO_ISR_EINT0 5 // the hardware priority of the interrupt

// Pines del periferico ADC
#define SBIT_BURST 16u
#define SBIT_START 24u
#define SBIT_PDN 21u
#define SBIT_EDGE 27u
#define SBIT_DONE 31u
#define SBIT_RESULT 4u
#define SBIT_CLCKDIV 8u

/* Prototipos de funciones */
static void vReceiverTask(void *pvParameters);
static void vSensorTask(void *pvParameters);
static void vUserTask(void *pvParameters);
static void vErrorTask(void *pvParameters);
static void vToggleLED(void);
void vStringGen(int, int, char *);
void UART3_Init(void);
void UART_Send(char *datos, int size);

/* Declare a variable of type QueueHandle_t to hold the handle of the queue being created. */
QueueHandle_t xPointerQueue;

TaskHandle_t xHandle;

/*Estructura del ejemplo del libro para el envio de mensaje de usuario */
struct msg_struct
{
	char *msg;
	char num;
	int len_str;
};

/**
 * @brief 
 * 
 * @return int 
 */
int main(void)
{
	/* P0_22 for the LED. */
	LPC_SC->EXTINT = (1 << SBIT_EINT0);		   /* Clear Pending interrupts */
	LPC_PINCON->PINSEL4 = (1 << PINSEL_EINT0); /* Configure P2_10 EINT0 */
	LPC_SC->EXTMODE = (1 << SBIT_EXTMODE0);	/* Configure EINTx as Edge Triggered*/
	LPC_SC->EXTPOLAR = (1 << SBIT_EXTPOLAR0);  /* Configure EINTx as Falling Edge */

	LPC_PINCON->PINSEL1 &= (~(3 << 12));
	LPC_GPIO0->FIODIR |= (1 << mainLED_BIT);
	NVIC_EnableIRQ(EINT0_IRQn); /* Enable the EINT0,EINT1 interrupts */

	LPC_SC->PCONP |= (1 << 12);								  /* Enable CLOCK for internal ADC controller */
	LPC_ADC->ADCR = ((1 << SBIT_PDN) | (10 << SBIT_CLCKDIV)); //Set the clock and Power ON ADC module
	LPC_PINCON->PINSEL1 |= 0x01 << 14;						  /* Select the P0_23 AD0[0] for ADC function */

	/* Enable traceanalycer snapshot */
	vTraceEnable(TRC_START);

	/* Create a queue that can hold a maximum of MAX_POINTERS pointers*/
	/* Cola de 10 punteros a struct */
	xPointerQueue = xQueueCreate(MAX_POINTERS, sizeof(struct msg_struct *));

	if (xPointerQueue != NULL)
	{
		xTaskCreate(vReceiverTask, "Transmisor", configMINIMAL_STACK_SIZE, NULL, RECEIVE_TASK_PRIORITY, NULL);
		xTaskCreate(vSensorTask, "Sensor_LDR", configMINIMAL_STACK_SIZE, NULL, PERIODIC_TASK_PRIORITY, NULL);
		xTaskCreate(vUserTask, "Usuario", configMINIMAL_STACK_SIZE, NULL, APERIODIC_TASK_PRIORITY, &xHandle);
		vTaskStartScheduler();
	}
	else
	{
		xTaskCreate(vErrorTask, "ERROR", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
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
static void vErrorTask(void *pvParameters)
{
	for (;;)
	{
		vToggleLED();
		vTaskDelay(750);
	}
}

/**
 * @brief Tarea de envio periodico de lectura de un fotoresistor. Se realiza la
 *        conversion ADC del canal 0 del ADC0, enviando el valor obtenido a la tarea
 *        central.
 * 
 * @param pvParameters 
 */
static void vSensorTask(void *pvParameters)
{
	uint16_t adc_result;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	/* Define a task that performs an action every 50 milliseconds. */
	const TickType_t xPeriod = pdMS_TO_TICKS(50);

	struct msg_struct *mensaje = pvPortMalloc(sizeof(struct msg_struct *));

	/* Seleccionamos canal 0 del ADC */
	LPC_ADC->ADCR |= 0x01;

	/* Enter the loop that defines the task behavior. */
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		/* Iniciamos la conversion ADC */
		LPC_ADC->ADCR |= 1 << SBIT_START;
		/* Esperamos hasta que la conversion este completa */
		while ((LPC_ADC->ADGDR >> SBIT_DONE) == 0)
			;
		/*Leemos los 12bits del ADC*/
		adc_result = (LPC_ADC->ADGDR >> SBIT_RESULT) & 0xFFF;

		mensaje->msg = NULL;
		mensaje->num = adc_result;

		xQueueSend(xPointerQueue, &mensaje, pdMS_TO_TICKS(1));

		/*Free a estructura*/
		vPortFree(mensaje);
	}
}

/**
 * @brief 
 * 
 * @param pvParameters 
 */
static void vUserTask(void *pvParameters)
{
	uint8_t random_A = 0;
	uint8_t random_B = 0;
	//uint32_t ulNotifiedValue;
	//portTickType xLastWakeTime;

	char *string_concat = pvPortMalloc(MAX_STRING);
	/*Aloco estructura donde colocara cada string generado para colocar en la cola*/
	struct msg_struct *mensaje = pvPortMalloc(sizeof(struct msg_struct *));
	mensaje->msg = pvPortMalloc(sizeof(char *));
	//Para limpiar
	mensaje->num = 0;

	//xTaskNotifyWait(0x01, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
	for (;;)
	{
		//if ((ulNotifiedValue & 0x01) != 0)
		//{
		random_A = (rand() + xTaskGetTickCount()) % (7);
		random_B = (rand() + xTaskGetTickCount()) % (7);
		/* Send to the queue - causing the queue receive task to flash its LED.*/
		vStringGen(random_A, random_B, string_concat);
		strcpy(mensaje->msg, string_concat);
		mensaje->len_str = strlen(mensaje->msg);

		xQueueSend(xPointerQueue, &mensaje, 2);

		/*Free a estructura*/
		vPortFree(mensaje->msg);
		vPortFree(string_concat);
		/* toggle LED 22 */
		vToggleLED();
		//}
		//xTaskNotifyWait(0x01, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
		vTaskSuspend(NULL);
	}
}

/**
 * Tarea de recepcion. Se fija en la cola si hay algo y lo recibe. Compara campos de la estructura, y lo
 * reenvia por puerto serie.
 *
 *
 * @param void*  pvParameters
 */
static void vReceiverTask(void *pvParameters)
{
	struct msg_struct *receptor; //creo puntero a estructura
	char buffer[TMP_INT];

	/*Modulo UART*/
	UART3_Init();

	for (;;)
	{
		/* Receive the address of a buffer. */
		if (xQueueReceive(xPointerQueue, /* The handle of the queue. */
						  &receptor,	 /* Store the buffer’s address in pcReceivedString. */
						  portNEW_DELAY) == pdTRUE)
		{
			if (receptor->msg == NULL)
			{ //Enviar string de tarea periodica
				/*Convierto valor recibido, a base 10, y lo pongo en tmp buf*/
				itoa(receptor->num, buffer, 10);
				/*Concateno \r\n*/
				strcat(buffer, "\r\n");
				/*Mando la temperatura recibida a traves del puerto UART*/
				UART_Send(buffer, strlen(buffer));
			}
			else
			{
				//Enviar string de tarea aperiodica
				UART_Send(receptor->msg, receptor->len_str);
			}
			/*Free a estructura*/
			vPortFree(receptor);
		}
	}
}

/**
 * @brief 
 * 
 * @param str_length 
 * @return char* 
 */
void vStringGen(int random_A, int random_B, char *string_concat)
{
	static char *sel_A[7] = {
		"Alianza Ninja",
		"Frente Populas",
		"Frente Hipster",
		"Movimiento Vegano",
		"Frente Tuitero",
		"Frente Pacifista",
		"Movimiento Aguafiestas"};

	static char *sel_B[7] = {
		" del Vaticano",
		" Democrata",
		" del Futuro",
		" de Hogwarts",
		" del Mal",
		" Dialogante",
		" de Invernalia"};

	strcat(string_concat, sel_A[random_A]);
	strcat(string_concat, sel_B[random_B]);
	strcat(string_concat, "\r\n");
}

/**
 * @brief 
 * 
 */
static void vToggleLED(void)
{
	unsigned long ulLEDState;
	/* Obtain the current P0 state. */
	ulLEDState = LPC_GPIO0->FIOPIN;
	/* Turn the LED off if it was on, and on if it was off. */
	LPC_GPIO0->FIOCLR = ulLEDState & (1 << mainLED_BIT);
	LPC_GPIO0->FIOSET = ((~ulLEDState) & (1 << mainLED_BIT));
}

/**
 * @brief 
 * 
 */
void UART3_Init(void)
{
	/*UART3, chau UART1*/
	LPC_SC->PCONP |= (1 << 25);
	/*Deshabilito tambien UART0 y UART1*/
	LPC_SC->PCONP &= ~(3 << 3);
	/*PCLKSEL1*/
	LPC_SC->PCLKSEL1 |= (1 << 18);
	/*8 bits word*/
	LPC_UART3->LCR = 0x03;
	/*Bit de stop*/
	LPC_UART3->LCR |= (1 << 2);
	/*Config*/
	LPC_UART3->LCR |= 0b10000000;
	LPC_UART3->DLL = 54; //*U3LCR 0b10100001 ; // 115200
	LPC_UART3->DLM = 0;  //*U3LCR
	LPC_UART3->LCR &= ~(1 << 7);
	//pin 0 TXD0 pin 1 RXD0 P0
	LPC_PINCON->PINSEL0 = 0b1010; // *PINSEL0 configurar los pines port 0
	LPC_PINCON->PINMODE0 = 0;	 // *PINMODE0 pin a pull up
}

/**
 * @brief 
 * 
 * @param data 
 * @param size 
 */
void UART_Send(char *data, int size)
{
	for (int i = 0; i < size; i++)
	{
		while ((LPC_UART3->LSR & (1 << 5)) == 0)
		{
		}						  //*U3LSR // Wait for Previous transmission
		LPC_UART3->THR = data[i]; //*U3THR
	}
}

/**
 * @brief 
 * 
 */
/**
 * Funcion Aperiodica representa el ingreso de caracteres de un usuario. Se genera una cadena de longitud
 * variable, de manera aleatoria, y se envia aperiodicamente (i,e cada cierto tiempo que tambien es variable)
 *
 * @param: void*  pvParameters
 *
 */
void EINT0_IRQHandler(void)
{
	//taskDISABLE_INTERRUPTS();
	BaseType_t xHigherPriorityTaskWoken;
	//Bajo bandera de interrupcion
	LPC_SC->EXTINT = 1 << 0;

	xHigherPriorityTaskWoken = pdFALSE;

	BaseType_t xYieldRequired;

	// Resume the suspended task.
	xYieldRequired = xTaskResumeFromISR(xHandle);
	if (xYieldRequired == pdTRUE)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	//taskENABLE_INTERRUPTS();
	//xTaskNotifyFromISR(xHandle, 0x01, eSetBits, xHigherPriorityTaskWoken);
	//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 
 * 
 */
void vConfigureTimerForRunTimeStats(void)
{
	const unsigned long TCR_COUNT_RESET = 2, CTCR_CTM_TIMER = 0x00,
						TCR_COUNT_ENABLE = 0x01;

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

/*
 * Necessary functions for FreeRTOS
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	/* This function will get called if a task overflows its stack. */
	(void)pxTask;
	(void)pcTaskName;
	for (;;)
		;
}
