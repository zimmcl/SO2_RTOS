/**
 * @file B_3Tasks_UserSensorCons_S.c
 * @author Ezequiel Zimmel (ezequielzimmel@gmail.com)
 * @brief Programa que simula la interaccion de tres actores, dos de ellos
 * 		  productores, un sensor de recurrencia periórica y la interacion de 
 * 		  un usuario con recurrencia apariódica, y el restante es un consumidor
 * 		  que envía los datos obtenidos por UART.
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
#include "semphr.h"

/* Priorities at which the tasks are created. */
#define RECEIVE_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define PERIODIC_TASK_PRIORITY (tskIDLE_PRIORITY + 4)
#define APERIODIC_TASK_PRIORITY (tskIDLE_PRIORITY + 6)
#define REPRIORITIZE_TASK_PRIORITY (tskIDLE_PRIORITY + configMAX_PRIORITIES)
/* The bit of port 0 that the LPCXpresso LPC13xx LED is connected. */
#define mainLED_BIT (22)
/* Cantidad maxima de elementos en la cola. */
#define MAX_POINTERS 5

/* The rate at which data is sent to the queue, specified in milliseconds. */
#define PERIODIC_SEND_FREQUENCY_MS (50 / portTICK_RATE_MS)

//Defines del programa
#define MAX_STRING 64
#define TMP_INT 5
#define portNEW_DELAY 200

/* Prototipos de funciones */
static void vReceiverTask(void *pvParameters);
static void vPeriodicSensorTask(void *pvParameters);
static void vAperiodicUserTask(void *pvParameters);
static void vReprioritizeTask(void *pvParameters);
static void vErrorTask(void *pvParameters);
static void vToggleLED(void);
char *vStringGen(uint8_t);
void UART3_Init(void);
void UART_Send(char *datos, int size);

/* Declare a variable of type QueueHandle_t to hold the handle of the queue being created. */
QueueHandle_t xQueue = NULL;
/* Create a mutex type semaphore. */
SemaphoreHandle_t xSemaphore = NULL;
/* Manejadores de las tareas. */
TaskHandle_t xHandleTransmisor;
TaskHandle_t xHandleSensor;
TaskHandle_t xHandleUser;

/* Estructura para el envio de mensajes. Si el mensaje proviene del
   sensor se modifica el campo num, por otro lado si el mensaje proviene
   del usuario se modifica el cambio msg y la longitud de dicho mensaje.*/
struct msg_struct
{
	char *msg;
	char num;
	int len_str;
};

/**
 * @brief Reserva memoria en la creación de la cola y el semaforo. Inicializa el
 * 		  snap shot del tracealyzer. Luego crea las tareas y las planifica.
 * 		  A las tareas se le asigna su correspondiente handler para poder modificar
 *        su comportamiento.
 * 
 * @return int 
 */
int main(void)
{
	/* P0_22 for the LED. */
	LPC_PINCON->PINSEL1 &= (~(3 << 12));
	LPC_GPIO0->FIODIR |= (1 << mainLED_BIT);

	/* Enable traceanalycer snapshot */
	vTraceEnable(TRC_START);

	/* Create a queue that can hold a maximum of MAX_POINTERS pointers*/
	/* Cola de 5 punteros a struct */
	xQueue = xQueueCreate(MAX_POINTERS, sizeof(struct msg_struct *));
	/* The mutexes are the better choice for implementing simple mutual exclusion. */
	xSemaphore = xSemaphoreCreateMutex();

	if ((xQueue != NULL) && (xSemaphore != NULL))
	{
		xTaskCreate(vReceiverTask, "Transmisor", configMINIMAL_STACK_SIZE, NULL, RECEIVE_TASK_PRIORITY, &xHandleTransmisor);
		xTaskCreate(vPeriodicSensorTask, "Sensor_LDR", configMINIMAL_STACK_SIZE, NULL, PERIODIC_TASK_PRIORITY, &xHandleSensor);
		xTaskCreate(vAperiodicUserTask, "Usuario", configMINIMAL_STACK_SIZE, NULL, APERIODIC_TASK_PRIORITY, &xHandleUser);
		xTaskCreate(vReprioritizeTask, "Reprioritize", configMINIMAL_STACK_SIZE, NULL, REPRIORITIZE_TASK_PRIORITY, NULL);
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
 * @brief Tarea que se ejecuta cada 5 segundos alterando las prioridades
 *        de las demás tareas. El valor de prioridad asignado y aleatorio
 * 		  pero siempre mayor que la prioridad tskIDLE_PRIORITY y menor a
 *   	  configMAX_PRIORITIES (FreeRTOSConfig.h). Se garantiza que las
 * 		  prioridades sean diferentes entre sí.
 * 
 * @param pvParameters 
 */
static void vReprioritizeTask(void *pvParameters)
{
	UBaseType_t xNewPriority[3];
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	traceString cht = xTraceRegisterString("Reprioritize");
	const TickType_t xPeriod = pdMS_TO_TICKS(5000);

	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		vTracePrint(cht, "T_Reprioritize");
		do
		{
			for (int i = 0; i < 3; i++)
			{
				xNewPriority[i] = ((rand() % (configMAX_PRIORITIES - 3)) + 1);
			}
		} while ((xNewPriority[0] == xNewPriority[1]) &&
				 (xNewPriority[0] == xNewPriority[2]) &&
				 (xNewPriority[1] == xNewPriority[2]));

		vTaskPrioritySet(xHandleTransmisor, xNewPriority[0]);
		vTaskPrioritySet(xHandleSensor, xNewPriority[1]);
		vTaskPrioritySet(xHandleUser, xNewPriority[2]);
	}
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
 * @brief Tarea periodica representando el sensor. Se genera un valor entre 0 y 255,
 * 		  que emula la medicion realiza por el sensor. La periodicidad de la tarea es
 * 		  de 20ms.
 * 		  Estando la tarea en ejecucion para poder realizar la "medicion" y luego el
 * 		  envío del dato a la cola, necesita de tener acceso al mutex. 
 * 
 * @param pvParameters 
 */
static void vPeriodicSensorTask(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	/* Define a task that performs an action every 20 milliseconds. */
	const TickType_t xPeriod = pdMS_TO_TICKS(20);

	/* Reservo memoria para la estructura que contendra el mensaje (num) a enviar. */
	struct msg_struct *mensaje = pvPortMalloc(sizeof(struct msg_struct *));

	/* Enter the loop that defines the task behavior. */
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		if (xSemaphore != NULL)
		{
			/* Si el semaforo esta libre, lo tomo. */
			if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(2)) == pdTRUE)
			{
				mensaje->msg = NULL;
				/* Genero valor aleatorio de 0 a 255 y lo casteo al tipo del campo. */
				mensaje->num = (char)(rand() % 255);

				/* Espera de 1ms hasta que la cola disponga de lugar. */
				xQueueSend(xQueue, &mensaje, pdMS_TO_TICKS(1));

				/* Libero la memoria reservada. */
				vPortFree(mensaje);
				/* Libero el semaforo. */
				xSemaphoreGive(xSemaphore);
			}
		}
	}
}

/**
 * @brief  Tarea aperiodica que representa el ingreso de caracteres de un usuario. Se genera un valor
 * 		   entero aleatoria que sera empleado en la generacion de una cadena de longitud variable. La
 * 		   tarea puede ejecutarse cada [100 - 1000]ms generando la aperiodicidad.
 * 
 * @param pvParameters 
 */
static void vAperiodicUserTask(void *pvParameters)
{
	portTickType xLastWakeTime;
	/* Reservo memoria para la estructura que contendra el mensaje a enviar. */
	struct msg_struct *mensaje = pvPortMalloc(sizeof(struct msg_struct *));
	mensaje->msg = pvPortMalloc(sizeof(char *));
	mensaje->num = 0;
	uint8_t xRandomLen = 0;

	xLastWakeTime = xTaskGetTickCount();
	/* Define a task that performs an action every random milisecons starting*/
	TickType_t xPeriod = pdMS_TO_TICKS((rand() % (900)) + 100); //100-1000 ms

	for (;;)
	{
		xPeriod = pdMS_TO_TICKS((rand() % (900)) + 100); //100-1000 ms
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		if (xSemaphore != NULL)
		{
			/* Si el semaforo esta libre, lo tomo. */
			if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(2)) == pdTRUE)
			{
				/* Genero valor aleatorio empleado para la generacion de la cadena de char. */
				xRandomLen = (rand() + xTaskGetTickCount()) % (MAX_STRING - 3);

				/* Obtengo la cadena aleatorio y la almaceno en el campo msg. */
				mensaje->msg = vStringGen(xRandomLen);
				mensaje->len_str = xRandomLen;

				/* Espera de 10ms hasta que la cola disponga de lugar. */
				xQueueSend(xQueue, &mensaje, pdMS_TO_TICKS(10));
				vToggleLED();
				/* Libero la memoria reservada. */
				vPortFree(mensaje->msg);
				/* Libero el semaforo. */
				xSemaphoreGive(xSemaphore);
			}
		}
	}
}

/**
 * @brief Tarea de recepcion. Se fija en la cola si hay algo y luego lo retrasmite por UART3. Para determinar que
 * 		  tipo de elemento hay en la cola, realiza la comparacion en los campos de la estructura.
 * 
 * @param pvParameters 
 */
static void vReceiverTask(void *pvParameters)
{
	//portTickType xLastWakeTime;
	struct msg_struct *receptor = pvPortMalloc(sizeof(struct msg_struct *));
	char buffer[TMP_INT];
	//xLastWakeTime = xTaskGetTickCount();
	//TickType_t xPeriod = pdMS_TO_TICKS(5);
	/* Inicializo periferico UART3. */
	UART3_Init();

	for (;;)
	{
		//vTaskDelayUntil(&xLastWakeTime, xPeriod);
		/* Verifica si hay algo en la cola. Espera hasta 200ms por algún mensaje. */
		if (xQueueReceive(xQueue, &receptor, portNEW_DELAY) == pdTRUE)
		{
			/* Como la tarea aperiódica es menos frecuente, verifica en primer lugar si
			   existe algun mensaje generado por el usuario en la cola. */
			/* Enviar string de tarea periodica. */
			if (receptor->msg == NULL)
			{
				/* Convierto valor recibido, a base 10, y lo pongo en buffer. */
				itoa(receptor->num, buffer, 10);
				strcat(buffer, "\r\n");
				/* Mando valor sensado recibido a traves del periferico UART3. */
				UART_Send(buffer, strlen(buffer));
			}
			/* Enviar string de tarea aperiodica */
			else
			{
				/* Mando texto enviado por el usuario a traves del periferico UART3. */
				UART_Send(receptor->msg, receptor->len_str);
			}
			/*Free a estructura*/
			vPortFree(receptor);
		}
	}
}

/**
 * @brief Emplea el valor aleatorio pasado por valor en la funcion para así generar
 * 		  una cadena de longitud variable.
 * 
 * @param str_length Tamaño de la cadena a generar.
 * @return char* Cadena aleatorio generada.
 */
char *vStringGen(uint8_t xRandomLen)
{
	static char *xCharSet = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

	char *xRandomString = pvPortMalloc(xRandomLen += 3);

	/* Itera sobre la longitud de la cadena a generar. */
	for (int i = 0; i < xRandomLen - 3; i++)
	{
		/* Selecciona de manera aleatoria un elemento del xCharSet para ir
		   armando la cadena de salida. */
		xRandomString[i] = xCharSet[rand() % MAX_STRING];
	}
	return xRandomString;
}

/**
 * @brief Conmuta el estado del led integrado a la placa, pin P0.22, entre
 *        encendido/apagado.
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
 * @brief Inicializa el periferico UART3
 * 
 */
void UART3_Init(void)
{
	/* UART 3 power/clock control bit */
	LPC_SC->PCONP |= (1 << 25);
	/* Deshabilito UART0 y UART1. */
	LPC_SC->PCONP &= ~(3 << 3);
	/* Peripheral clock selection for UART3. */
	LPC_SC->PCLKSEL1 |= (1 << 18);
	/* 8 bits word. */
	LPC_UART3->LCR = 0x03;
	/* Bit de stop. */
	LPC_UART3->LCR |= (1 << 2);
	/* Configuracion general del periferico. */
	LPC_UART3->LCR |= 0b10000000; // Enable access to Divisor Latches
	/* Baudrate 115200 */
	LPC_UART3->DLL = 54; // U3LCR 0b10100001 
	LPC_UART3->DLM = 0;  // U3LCR
	/* Disable access to Divisor Latches. */
	LPC_UART3->LCR &= ~(1 << 7);
	/* Pin P0.0 TXD3 - Pin P0.1 RXD3. */
	LPC_PINCON->PINSEL0 = 0b1010;
	/* Pin pull up. */
	LPC_PINCON->PINMODE0 = 0;	 
}

/**
 * @brief Envio de datos por periferico UART3
 * 
 * @param data mensaje a enviar
 * @param size tamaño del mensaje
 */
void UART_Send(char *data, int size)
{
	for (int i = 0; i < size; i++)
	{
		while ((LPC_UART3->LSR & (1 << 5)) == 0)
		{
		}
		LPC_UART3->THR = data[i];
	}
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
