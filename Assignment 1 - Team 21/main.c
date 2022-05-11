#include "FreeRTOS.h"
#include "TM4C123GH6PM.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"

#define mainSW_INTERRUPT_ID		( ( IRQn_Type ) 0 )
#define mainTRIGGER_INTERRUPT()	NVIC_SetPendingIRQ( mainSW_INTERRUPT_ID )
#define mainCLEAR_INTERRUPT()	NVIC_ClearPendingIRQ( mainSW_INTERRUPT_ID )
#define clear_ICR() (GPIOA->ICR)|=(0x02)
#define mainSOFTWARE_INTERRUPT_PRIORITY 		( 5 )

#define BAUD (115200)


static void vInitTask( void *pvParameters );
static void vHandlerTask( void *pvParameters );
static void vPeriodicTask( void *pvParameters );
void printstring(char * string);

static void prvSetupSoftwareInterrupt( void );
void vSoftwareInterruptHandler( void ); 

/*-----------------------------------------------------------*/

xSemaphoreHandle xBinarySemaphore;
xTaskHandle xInitTaskHandle;

/*-----------------------------------------------------------*/

int main( void )
{
    vSemaphoreCreateBinary( xBinarySemaphore );
    if( xBinarySemaphore != NULL )
    {
      	prvSetupSoftwareInterrupt();
				xTaskCreate( vInitTask, "Init", configMINIMAL_STACK_SIZE, NULL, 5, &xInitTaskHandle );
        xTaskCreate( vPeriodicTask, "Periodic", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
			xTaskCreate( vHandlerTask, "Periodic", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
        vTaskStartScheduler();
    }
}
static void vInitTask( void *pvParameters )
{
	HWREG(SYSCTL_RCGCUART) |= 0x01;
	HWREG(SYSCTL_RCGCGPIO) |= 0x01;
	HWREG(GPIO_PORTA_BASE + GPIO_O_AFSEL) |= 0x03;
	HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) |= 0x11;
	HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) |= 0x03;
	HWREG(UART0_BASE + UART_O_CTL) &= ~0X01;
	HWREG(UART0_BASE + UART_O_IBRD) = 50000000 / 16 / BAUD;
	HWREG(UART0_BASE + UART_O_FBRD) = ((64 * ((50000000 / 16) % BAUD)) + BAUD / 2) / BAUD;
	HWREG(UART0_BASE + UART_O_LCRH) = (0x3 << 5);
	HWREG(UART0_BASE + UART_O_CC) = (0x00);
	HWREG(UART0_BASE + UART_O_CTL) = 0x301;
	
	vTaskSuspend(xInitTaskHandle);
}

static void vPeriodicTask( void *pvParameters )
{
    for( ;; )
    {
        vTaskDelay( 500 / portTICK_RATE_MS );

				printstring( "Periodic task - About to generate an interrupt." );
        mainTRIGGER_INTERRUPT();
        printstring( "Periodic task - Interrupt generated." );

    }
}
static void vHandlerTask( void *pvParameters )
{
    /* As per most tasks, this task is implemented within an infinite loop.

    Take the semaphore once to start with so the semaphore is empty before the
    infinite loop is entered.  The semaphore was created before the scheduler
    was started so before this task ran for the first time.*/
    xSemaphoreTake( xBinarySemaphore, 0 );

    for( ;; )
    {
        /* Use the semaphore to wait for the event.  The task blocks
        indefinitely meaning this function call will only return once the
        semaphore has been successfully obtained - so there is no need to check
        the returned value. */
        xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );

        /* To get here the event must have occurred.  Process the event (in this
        case we just print out a message). */
        printstring( "Handler task - Processing event.\n" );
    }
}
void GPIOA_Handler( void )
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	 xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );
    mainCLEAR_INTERRUPT();
		clear_ICR();
	printstring( "Interrupt Handler." );
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/
static void prvSetupSoftwareInterrupt( void )
{
	NVIC_SetPriority( mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY );

	NVIC_EnableIRQ( mainSW_INTERRUPT_ID );
}
/*-----------------------------------------------------------*/


void vApplicationMallocFailedHook( void )
{
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
}

void UARTprint(char c)
{
  // make sure previous transition has ended TXFF not full (0)
  while ((HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFF) != 0){};
	HWREG(UART0_BASE + UART_O_DR) = c;
}
void printstring(char * string){
	while(*string){
		UARTprint(*(string++));
	}
	UARTprint('\n');
	UARTprint('\r');
}
